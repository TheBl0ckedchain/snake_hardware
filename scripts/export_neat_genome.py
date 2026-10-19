#!/usr/bin/env python3
"""Export a trained NEAT genome (pickle) to a plain JSON network that the ROS nn_controller can run
without neat-python, then check that both give the same outputs.

    ~/snake_env/bin/python scripts/export_neat_genome.py \
        6_joint_snake/stage3_full.pkl 6_joint_snake/config-feedforward.txt \
        ros2_ws/src/snakebot_gait/config/nn_policy.json

Needs neat-python 0.92 (the version the genome was pickled with): ~/snake_env/bin/pip install neat-python==0.92
The JSON lists the nodes in evaluation order; each node computes
    value = tanh(2.5 * (bias + response * sum(w * value_in)))        (neat's tanh_activation)
"""
import json
import pickle
import random
import sys

import neat
from neat.graphs import feed_forward_layers


def main(genome_path, config_path, out_path):
    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation, config_path)
    with open(genome_path, 'rb') as f:
        genome = pickle.load(f)
    gc = config.genome_config
    for node in genome.nodes.values():
        if node.activation != 'tanh' or node.aggregation != 'sum':
            sys.exit(f'node {node.key}: only tanh/sum nodes are supported, got {node.activation}/{node.aggregation}')

    enabled = [c.key for c in genome.connections.values() if c.enabled]
    layers = feed_forward_layers(gc.input_keys, gc.output_keys, enabled)
    nodes = []
    for layer in layers:
        for key in sorted(layer):
            n = genome.nodes[key]
            links = [[i, genome.connections[(i, o)].weight] for (i, o) in enabled if o == key]
            nodes.append({'key': key, 'bias': n.bias, 'response': n.response, 'links': links})
    out = {
        'source': genome_path,
        'genome_key': genome.key,
        'fitness': genome.fitness,
        'input_keys': gc.input_keys,     # -1..-30: lidar beams, -31/-32: goal direction (dx, dy)
        'output_keys': gc.output_keys,   # 0: gait frequency, 1: head bias
        'nodes': nodes,
    }
    with open(out_path, 'w') as f:
        json.dump(out, f, indent=1)

    # Check: the JSON network must match neat's on random inputs.
    sys.path.insert(0, 'ros2_ws/src/snakebot_gait')
    from snakebot_gait.neat_net import JsonNetwork
    ref = neat.nn.FeedForwardNetwork.create(genome, config)
    mine = JsonNetwork(out)
    rng = random.Random(0)
    worst = 0.0
    for _ in range(1000):
        x = [rng.random() for _ in range(30)] + [rng.uniform(-1, 1), rng.uniform(-1, 1)]
        worst = max(worst, max(abs(a - b) for a, b in zip(ref.activate(x), mine.activate(x))))
    print(f'genome {genome.key} (fitness {genome.fitness:.1f}): {len(nodes)} nodes, '
          f'{len(enabled)} enabled links -> {out_path}; max |diff| vs neat over 1000 inputs: {worst:.2e}')
    if worst > 1e-9:
        sys.exit('MISMATCH')


if __name__ == '__main__':
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    main(*sys.argv[1:])
