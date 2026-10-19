"""Feed-forward network exported from a NEAT genome by scripts/export_neat_genome.py (no neat-python needed)."""
import json
import math


class JsonNetwork:
    def __init__(self, spec):
        if isinstance(spec, str):
            with open(spec) as f:
                spec = json.load(f)
        self.spec = spec
        self.input_keys = spec['input_keys']
        self.output_keys = spec['output_keys']
        self.nodes = [(n['key'], n['bias'], n['response'], [(i, w) for i, w in n['links']])
                      for n in spec['nodes']]

    def activate(self, inputs):
        if len(inputs) != len(self.input_keys):
            raise ValueError(f'expected {len(self.input_keys)} inputs, got {len(inputs)}')
        values = dict(zip(self.input_keys, inputs))
        for key in self.output_keys:
            values.setdefault(key, 0.0)
        for key, bias, response, links in self.nodes:
            s = sum(values[i] * w for i, w in links)
            values[key] = math.tanh(max(-60.0, min(60.0, 2.5 * (bias + response * s))))
        return [values[k] for k in self.output_keys]
