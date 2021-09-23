#!/usr/bin/env python3

# https://github.com/thiagopbueno/pypddl-parser


import argparse

import sys
sys.path.append('../../../pypddl-parser/pypddl-parser')
from pddlparser import PDDLParser

#class PDDLToMDP:

 #   def __init__(self):
        #parser_pkg = importlib.import_module("pypddl-parser")
  #      pass


def parse():
    usage = 'python3 pddl_to_mdp.py <DOMAIN> <INSTANCE>'
    description = 'pypddl-parser is a PDDL parser built on top of ply.'
    parser = argparse.ArgumentParser(usage=usage, description=description)

    parser.add_argument('domain',  type=str, help='path to PDDL domain file')
    parser.add_argument('problem', type=str, help='path to PDDL problem file')

    return parser.parse_args()


if __name__ == '__main__':
    args = parse()

    domain  = PDDLParser.parse(args.domain)
    problem = PDDLParser.parse(args.problem)

    print(domain)
    print(problem)

    print("++++++++++++++++++++++++++++++")


