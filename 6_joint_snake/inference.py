import math
import neat
import pickle
import random
import numpy as np

import train
from train import (
    initialize_simulation,
    load_world,
    load_snake,
    perform_lidar_scan,
    filter_lidar,
    scale_masses_inertia,
    SCALE,
    STAGE_GOALS,
)

############################
GENOME_PATH = "/scratch/abhijitdas/snake_hybrid/myrun/stage3_full.pkl"
CHECKPOINT_PATH = None
TARGET_GENOME_KEY = 7945
CONFIG_PATH = "./config/config-feedforward.txt"
MAX_STEPS = 12000
SUBSTEPS = 10
############################

GOAL_POSITION = STAGE_GOALS["full"]


def load_genome():
    config = neat.Config(
        neat.DefaultGenome,
        neat.DefaultReproduction,
        neat.DefaultSpeciesSet,
        neat.DefaultStagnation,
        CONFIG_PATH,
    )

    if CHECKPOINT_PATH is not None:
        pop = neat.Checkpointer.restore_checkpoint(CHECKPOINT_PATH)
        if TARGET_GENOME_KEY in pop.population:
            genome = pop.population[TARGET_GENOME_KEY]
        else:
            genome = max(pop.population.values(), key=lambda g: g.fitness if g.fitness else -1e9)
    else:
        with open(GENOME_PATH, "rb") as f:
            genome = pickle.load(f)

    print(f"Genome {genome.key}, fitness={genome.fitness}")
    net = neat.nn.FeedForwardNetwork.create(genome, config)
    return net, genome


def run_inference():
    net, genome = load_genome()

    # Set goal globally so load_world beacon is correct
    train.GOAL_POSITION = GOAL_POSITION
    train.MAX_STEPS = MAX_STEPS

    # --- Exactly mirror eval_genomes ---
    p = initialize_simulation()
    _, obstacles, walls, _, _, _ = load_world(p)
    snake_id = load_snake(p)
    scale_masses_inertia(p, snake_id, SCALE)

    start_x, start_y, _ = p.getBasePositionAndOrientation(snake_id)[0]

    # Seed AFTER world/snake — matches training
    random.seed(genome.key)
    np.random.seed(genome.key)

    fitness = 0.0
    num_collisions = 0
    prev_goal_dist = math.hypot(GOAL_POSITION[0] - start_x, GOAL_POSITION[1] - start_y)
    best_distance = prev_goal_dist
    phi_values = []
    goal_reached = False
    current_x, current_y = start_x, start_y

    HEAD_LIMIT = math.radians(45)

    for step in range(MAX_STEPS):
        lidar_distances = perform_lidar_scan(p, snake_id)
        lidar_filtered = filter_lidar(lidar_distances)

        current_x, current_y, _ = p.getBasePositionAndOrientation(snake_id)[0]

        dx = GOAL_POSITION[0] - current_x
        dy = GOAL_POSITION[1] - current_y
        dist_norm = np.linalg.norm([dx, dy]) + 1e-6
        dx /= dist_norm
        dy /= dist_norm

        inputs = lidar_filtered + [dx, dy]
        output = net.activate(inputs)

        w_min, w_max = 0.4, 2.0
        w = w_min + (output[0] + 1.0) * 0.5 * (w_max - w_min)

        phi0 = float(np.clip(output[1], -1.0, 1.0) * HEAD_LIMIT)
        phi_values.append(phi0)

        num_joints = p.getNumJoints(snake_id)
        count = 0
        for j in range(num_joints):
            joint_name = p.getJointInfo(snake_id, j)[1].decode("utf-8")
            if not joint_name.startswith("servo_joint"):
                continue
            if count == 0:
                joint_position = 0.3 * math.sin(w * step) + phi0
                joint_position = float(np.clip(joint_position, -HEAD_LIMIT, HEAD_LIMIT))
                p.setJointMotorControl2(
                    snake_id, j, p.POSITION_CONTROL,
                    targetPosition=joint_position,
                    force=200.0, maxVelocity=6.0,
                )
            else:
                joint_position = 1.0 * math.sin(w * step - count * (math.pi / 6))
                p.setJointMotorControl2(
                    snake_id, j, p.POSITION_CONTROL,
                    targetPosition=joint_position,
                    force=2500.0, maxVelocity=6.0,
                )
            count += 1

        collision_detected = False
        wall_detected = False
        for _ in range(SUBSTEPS):
            p.stepSimulation()
            if not collision_detected:
                for obs_id in obstacles:
                    if p.getContactPoints(snake_id, obs_id):
                        collision_detected = True
                        break
            if not wall_detected:
                for wall_id in walls:
                    if p.getContactPoints(snake_id, wall_id):
                        wall_detected = True
                        break
            if collision_detected or wall_detected:
                break

        if collision_detected or wall_detected:
            num_collisions += 1

        current_x, current_y, _ = p.getBasePositionAndOrientation(snake_id)[0]
        xy_distance = math.hypot(GOAL_POSITION[0] - current_x, GOAL_POSITION[1] - current_y)

        # Fitness (same as training)
        goal_progress = prev_goal_dist - xy_distance
        fitness += 10.0 * goal_progress
        prev_goal_dist = xy_distance

        if xy_distance < best_distance - 0.1:
            fitness += 50.0 * (best_distance - xy_distance)
            best_distance = xy_distance

        fitness -= 0.05

        if collision_detected or wall_detected:
            fitness -= 10.0

        if num_collisions > 15:
            fitness -= 100
            print(f"Step {step}: TERMINATED — too many collisions ({num_collisions})")
            break

        goal_radius = 1.5 * SCALE
        if xy_distance <= goal_radius:
            fitness += 3000 + 10000 + 2.0 * (MAX_STEPS - step)
            goal_reached = True
            print(f"Step {step}: GOAL REACHED! pos=({current_x:.2f}, {current_y:.2f}), fitness={fitness:.2f}")
            break

        if step % 200 == 0:
            print(f"Step {step:5d}: pos=({current_x:6.2f}, {current_y:6.2f}), dist={xy_distance:5.2f}, "
                  f"best={best_distance:5.2f}, collisions={num_collisions}, fitness={fitness:.1f}")

    mean_phi = np.mean(np.abs(phi_values)) if phi_values else 0.0
    std_phi = np.std(phi_values) if phi_values else 0.0

    print(f"\n{'='*60}")
    print(f"Genome {genome.key} | Goal: {goal_reached}")
    print(f"Final pos: ({current_x:.3f}, {current_y:.3f})")
    print(f"Steps: {step} | Collisions: {num_collisions}")
    print(f"Best distance: {best_distance:.2f}")
    print(f"Fitness: {fitness:.2f} (training reported: {genome.fitness:.2f})")
    print(f"Mean|phi|: {mean_phi:.3f} | Std phi: {std_phi:.3f}")
    print(f"{'='*60}")

    p.disconnect()


if __name__ == "__main__":
    run_inference()
