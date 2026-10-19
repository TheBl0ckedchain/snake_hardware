import math
import neat
import os
import random
import numpy as np
import logging
import argparse
import pickle

os.environ["PYBULLET_DEBUG"] = "0"
os.environ["PYBULLET_IGNORE_WARNINGS"] = "1"

# ============================================================
# Constants
# ============================================================
SCALE = 0.5
USE_MAXIMAL_COORDINATES = False
GRAVITY = -10
REAL_TIME_SIMULATION = 0
LIDAR_NUM_RAYS = 360
LIDAR_RAY_LENGTH = 6 * SCALE
SNAKE_URDF_PATH = "./urdf/snake.urdf"
OBSTACLE_URDF = "./urdf/cylinder.urdf"
PLANE_URDF = "plane.urdf"
WALL_URDF = "cube.urdf"

# 3-stage curriculum:
#   easy: ~3.5 units, past first obstacle row
#   mid:  ~8 units, through several obstacle rows  
#   full: ~16 units, the real goal
STAGE_GOALS = {
    "easy": [3 * SCALE, 8 * SCALE, 1 * SCALE],
    "mid":  [3 * SCALE, 18 * SCALE, 1 * SCALE],
    "full": [3 * SCALE, 33 * SCALE, 1 * SCALE],
}

STAGE_MAX_STEPS = {
    "easy": 5000,
    "mid":  8000,
    "full": 12000,   # was 5000 — genome 8301 reached Y=18.2 but ran out of time
}

# Set by main()
GOAL_POSITION = STAGE_GOALS["easy"]
MAX_STEPS = STAGE_MAX_STEPS["easy"]

save_dir = ""
save = ""


# ============================================================
# Simulation helpers
# ============================================================
def initialize_simulation():
    import pybullet
    import pybullet_utils.bullet_client as bc
    import pybullet_data
    p = bc.BulletClient(connection_mode=pybullet.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, GRAVITY)
    p.setRealTimeSimulation(REAL_TIME_SIMULATION)
    return p


def create_goal_beacon(p):
    collision_shape = p.createCollisionShape(
        p.GEOM_CYLINDER, radius=0.3 * SCALE, height=2.0 * SCALE
    )
    visual_shape = p.createVisualShape(
        p.GEOM_CYLINDER, radius=0.35 * SCALE, length=2.0 * SCALE,
        rgbaColor=[0, 1, 0, 0.8], specularColor=[0.5, 1, 0.5]
    )
    goal_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=[GOAL_POSITION[0], GOAL_POSITION[1], GOAL_POSITION[2]]
    )
    return goal_id


def load_world(p):
    WALL_POSITIONS = [
        [[i, -1.5, 0.5] for i in range(-11, 12)] +
        [[i, 38.5, 0.5] for i in range(-11, 12)] +
        [[-11, i, 0.5] for i in range(-1, 39)] +
        [[12, i, 0.5] for i in range(-1, 39)]
    ]
    WALL_POSITIONS[0].append([12, -1.5, 0.5])
    WALL_POSITIONS[0].append([12, 38.5, 0.5])

    obstacle_positions = [
        [5, 30.5, 0.5], [9, 30.5, 0.5], [0, 30.5, 0.5], [-4, 31, 0.5],
        [-4, 27, 0.5], [2, 27, 0.5], [7, 27, 0.5],
        [-6.5, 24.5, 0.5], [-1, 24, 0.5], [5, 24, 0.5], [10, 24, 0.5],
        [-5, 21.25, 0.5], [-1.5, 20.5, 0.5], [2, 21.5, 0.5], [7, 21.5, 0.5],
        [-4.5, 18.25, 0.5], [4.5, 18.5, 0.5], [10, 18, 0.5],
        [-6, 16, 0.5], [-1.5, 14.5, 0.5], [2, 16, 0.5], [7, 16, 0.5],
        [10, 12.5, 0.5], [5, 12.5, 0.5], [1.5, 11.5, 0.5], [-5, 12, 0.5], [-9.5, 12, 0.5],
        [9, 7, 0.5], [5, 8, 0.5], [-2.5, 9, 0.5], [-7, 9, 0.5],
        [7, 2, 0.5], [1, 6, 0.5], [-3, 5, 0.5], [-7.5, 4.5, 0.5],
    ]

    plane_id = p.loadURDF(PLANE_URDF)
    obstacle_positions_scaled = [[x * SCALE, y * SCALE, z * SCALE] for (x, y, z) in obstacle_positions]
    wall_positions_scaled = [[x * SCALE, y * SCALE, z * SCALE] for (x, y, z) in WALL_POSITIONS[0]]

    obstacles = [p.loadURDF(OBSTACLE_URDF, pos, globalScaling=SCALE, useFixedBase=True)
                 for pos in obstacle_positions_scaled]
    walls = [p.loadURDF(WALL_URDF, pos, globalScaling=SCALE, useFixedBase=True)
             for pos in wall_positions_scaled]
    goal_id = create_goal_beacon(p)
    return plane_id, obstacles, walls, obstacle_positions_scaled, wall_positions_scaled, goal_id


def load_snake(p):
    rotation_quaternion = p.getQuaternionFromEuler([0, 0, -math.pi / 2])
    snake_id = p.loadURDF(
        SNAKE_URDF_PATH,
        [3 * SCALE, 1 * SCALE, 1 * SCALE],
        rotation_quaternion,
        useMaximalCoordinates=USE_MAXIMAL_COORDINATES,
        globalScaling=SCALE
    )
    return snake_id


def perform_lidar_scan(p, snake_uid):
    snake_head_pos = p.getBasePositionAndOrientation(snake_uid)[0]
    ray_from_positions = [list(snake_head_pos)] * LIDAR_NUM_RAYS
    ray_to_positions = [
        [
            snake_head_pos[0] + LIDAR_RAY_LENGTH * math.cos(2 * math.pi * i / LIDAR_NUM_RAYS),
            snake_head_pos[1] + LIDAR_RAY_LENGTH * math.sin(2 * math.pi * i / LIDAR_NUM_RAYS),
            1 * SCALE
        ]
        for i in range(LIDAR_NUM_RAYS)
    ]
    results = p.rayTestBatch(ray_from_positions, ray_to_positions)
    distances = [result[2] * LIDAR_RAY_LENGTH for result in results]
    return distances


def filter_lidar(lidar_distances):
    angles_of_interest = [i for i in range(LIDAR_NUM_RAYS) if i % 12 == 0]
    filtered_distances = [lidar_distances[i] for i in angles_of_interest]
    max_distance = LIDAR_RAY_LENGTH
    normalized_distances = [dist / max_distance for dist in filtered_distances]
    return normalized_distances


def scale_masses_inertia(p, body_id, scale: float):
    assert scale > 0.0
    s3 = scale ** 3
    s5 = scale ** 5
    n_links = p.getNumJoints(body_id)
    link_indices = [-1] + list(range(n_links))
    for link in link_indices:
        info = p.getDynamicsInfo(body_id, link)
        mass = info[0]
        inertia_diag = info[2]
        new_mass = mass * s3
        new_inertia = [ix * s5 for ix in inertia_diag]
        p.changeDynamics(
            bodyUniqueId=body_id, linkIndex=link,
            mass=new_mass, localInertiaDiagonal=new_inertia
        )


# ============================================================
# Genome evaluation
# ============================================================
def eval_genomes(genome, config):
    global save_dir, save

    net = neat.nn.FeedForwardNetwork.create(genome, config)

    p = initialize_simulation()
    _, obstacles, walls, _, _, _ = load_world(p)
    snake_id = load_snake(p)
    scale_masses_inertia(p, snake_id, SCALE)

    start_x, start_y, _ = p.getBasePositionAndOrientation(snake_id)[0]

    max_steps = MAX_STEPS
    fitness = 0.0
    substeps = 10
    GOAL_REWARD = 3000
    num_collisions = 0

    phi_values = []
    goal_reached = False

    prev_goal_dist = math.hypot(
        GOAL_POSITION[0] - start_x,
        GOAL_POSITION[1] - start_y
    )
    best_distance = prev_goal_dist

    random.seed(genome.key)
    np.random.seed(genome.key)

    current_x, current_y = start_x, start_y
    step = 0

    for step in range(max_steps):
        # ----- Observation -----
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

        # ----- Action -----
        w_min, w_max = 0.4, 2.0
        w = w_min + (output[0] + 1.0) * 0.5 * (w_max - w_min)

        HEAD_LIMIT = math.radians(45)
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
                    force=200.0, maxVelocity=6.0
                )
            else:
                joint_position = 1.0 * math.sin(w * step - count * (math.pi / 6))
                p.setJointMotorControl2(
                    snake_id, j, p.POSITION_CONTROL,
                    targetPosition=joint_position,
                    force=2500.0, maxVelocity=6.0
                )
            count += 1

        # ----- Sim step -----
        collision_detected = False
        wall_detected = False
        for _ in range(substeps):
            p.stepSimulation()
            if not collision_detected:
                for obstacle_id in obstacles:
                    if p.getContactPoints(snake_id, obstacle_id):
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
        xy_distance = math.hypot(
            GOAL_POSITION[0] - current_x,
            GOAL_POSITION[1] - current_y
        )

        # ===============================
        # FITNESS SHAPING
        # ===============================
        goal_progress = prev_goal_dist - xy_distance
        fitness += 10.0 * goal_progress
        prev_goal_dist = xy_distance

        if xy_distance < best_distance - 0.1:
            fitness += 50.0 * (best_distance - xy_distance)
            best_distance = xy_distance

        fitness -= 0.05

        if collision_detected or wall_detected:
            fitness -= 10.0

        # ----- Termination -----
        if num_collisions > 15:
            fitness -= 100
            break

        goal_radius = 1.5 * SCALE
        if xy_distance <= goal_radius:
            fitness += GOAL_REWARD + 10000
            fitness += 2.0 * (max_steps - step)
            goal_reached = True
            logging.info(
                f"[GENOME {genome.key}] GOAL reached | "
                f"Steps: {step} | Collisions: {num_collisions} | "
                f"Fitness: {fitness:.2f}"
            )
            break

    mean_phi = np.mean(np.abs(phi_values)) if phi_values else 0.0
    std_phi = np.std(phi_values) if phi_values else 0.0

    logging.info(
        f"[GENOME {genome.key}] END | "
        f"Fitness: {fitness:.2f} | Steps: {step} | "
        f"Final Y: {current_y:.3f} | BestDist: {best_distance:.2f} | "
        f"Collisions: {num_collisions} | Goal: {goal_reached} | "
        f"Mean|phi|: {mean_phi:.3f} | Std phi: {std_phi:.3f}"
    )

    p.disconnect()
    return fitness


# ============================================================
# Main
# ============================================================
def main(args):
    global GOAL_POSITION, MAX_STEPS, save_dir, save

    GOAL_POSITION = STAGE_GOALS[args.stage]
    MAX_STEPS = STAGE_MAX_STEPS[args.stage]

    base_dir = f"/scratch/abhijitdas/snake_hybrid/{args.save_dir}"
    os.makedirs(base_dir, exist_ok=True)

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        filename=f'{base_dir}/{args.save}.log',
        filemode='w'
    )
    logging.info(f"Stage: {args.stage} | Goal: {GOAL_POSITION} | MaxSteps: {MAX_STEPS}")

    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, "config", "config-feedforward.txt")
    config = neat.Config(
        neat.DefaultGenome, neat.DefaultReproduction,
        neat.DefaultSpeciesSet, neat.DefaultStagnation,
        config_path
    )

    if args.warm_start and os.path.exists(args.warm_start):
        print(f"Warm-starting from {args.warm_start}")
        population = neat.Checkpointer.restore_checkpoint(args.warm_start)
    else:
        population = neat.Population(config)

    population.add_reporter(neat.StdOutReporter(True))
    stats = neat.StatisticsReporter()
    population.add_reporter(stats)
    population.add_reporter(neat.Checkpointer(
        50,
        filename_prefix=f'{base_dir}/{args.save}-neat-parallel-ckpt-'
    ))

    num_generations = args.generations
    print(f"Stage: {args.stage} | Goal: {GOAL_POSITION} | MaxSteps: {MAX_STEPS}")
    print("cpu count:", os.cpu_count())
    parallel_evaluator = neat.ParallelEvaluator(os.cpu_count(), eval_genomes)
    winner = population.run(parallel_evaluator.evaluate, num_generations)

    out_path = f"{base_dir}/{args.save}.pkl"
    try:
        with open(out_path, "wb") as f:
            pickle.dump(winner, f)
    except Exception:
        with open(f"{args.save}.pkl", "wb") as f:
            pickle.dump(winner, f)

    print("Best genome fitness:", winner.fitness)
    logging.info("Training complete.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--save_dir", required=True, help="Training save dir")
    parser.add_argument("--save", required=True, help="Training save name")
    parser.add_argument("--stage", choices=["easy", "mid", "full"], default="easy",
                        help="Curriculum stage.")
    parser.add_argument("--generations", type=int, default=150)
    parser.add_argument("--warm_start", default=None,
                        help="Path to a NEAT checkpoint to resume from.")
    args = parser.parse_args()

    save_dir = args.save_dir
    save = args.save
    main(args)
