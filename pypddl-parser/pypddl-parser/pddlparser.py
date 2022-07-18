# This file is part of pypddl-parser.

# pypddl-parser is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# pypddl-parser is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with pypddl-parser.  If not, see <http://www.gnu.org/licenses/>.


from ply import lex
from ply import yacc

from term      import Term
from literal   import Literal
from predicate import Predicate
from reward    import Reward
from action    import Action
from domain    import Domain
from problem   import Problem
from constraint import Constraint

import traceback 

debug = False


tokens = (
    'INCREASE',##
    'REWARD',##
    'REWARD_VALUE',##
    'NAME',
    'VARIABLE',
    'PROBABILITY',
    'LPAREN',
    'RPAREN',
    'HYPHEN',
    'EQUALS',
    'DEFINE_KEY',
    'DOMAIN_KEY',
    'REQUIREMENTS_KEY',
    'STRIPS_KEY',
    'EQUALITY_KEY',
    'TYPING_KEY',
    'PROBABILISTIC_EFFECTS_KEY',
    'TYPES_KEY',
    'PREDICATES_KEY',
    'ACTION_KEY',
    'PARAMETERS_KEY',
    'PRECONDITION_KEY',
    'EFFECT_KEY',
    'AND_KEY',
    'OR_KEY',
    'NOT_KEY',
    'PROBABILISTIC_KEY',
    'PROBLEM_KEY',
    'OBJECTS_KEY',
    'INIT_KEY',
    'GOAL_KEY',
    'NEGATIVE_PRECONDITIONS_KEY', # Currently not used :(
    'CONSTRAINTS_KEY', ##########
    'FORALL_KEY', ###########
    'ATMOSTONCE_KEY', ##########
    'REWARDS_KEY',
    'DISJUNCTIVE_PRECONDITIONS_KEY',
    'CONDITIONAL_EFFECTS_KEY',
    'WHEN_KEY',
    'CONSTANTS_KEY'
)


t_LPAREN = r'\('
t_RPAREN = r'\)'
t_HYPHEN = r'\-'
t_EQUALS = r'='

t_ignore = ' \t'

reserved = {
    'define'                    : 'DEFINE_KEY',
    'domain'                    : 'DOMAIN_KEY',
    ':requirements'             : 'REQUIREMENTS_KEY',
    ':strips'                   : 'STRIPS_KEY',
    ':equality'                 : 'EQUALITY_KEY',
    ':typing'                   : 'TYPING_KEY',
    ':probabilistic-effects'    : 'PROBABILISTIC_EFFECTS_KEY',
    ':types'                    : 'TYPES_KEY',
    ':predicates'               : 'PREDICATES_KEY',
    ':action'                   : 'ACTION_KEY',
    ':parameters'               : 'PARAMETERS_KEY',
    ':precondition'             : 'PRECONDITION_KEY',
    ':effect'                   : 'EFFECT_KEY',
    'and'                       : 'AND_KEY',
    'or'                        : 'OR_KEY',
    'not'                       : 'NOT_KEY',
    'probabilistic'             : 'PROBABILISTIC_KEY',
    'problem'                   : 'PROBLEM_KEY',
    ':domain'                   : 'DOMAIN_KEY',
    ':objects'                  : 'OBJECTS_KEY',
    ':init'                     : 'INIT_KEY',
    ':goal'                     : 'GOAL_KEY',
    'increase'                  : 'INCREASE', 
    'reward'                    : 'REWARD', 
    ':constraints'              : 'CONSTRAINTS_KEY',
    'forall'                    : 'FORALL_KEY',
    'at-most-once'              : 'ATMOSTONCE_KEY',
    ':negative-preconditions'   : 'NEGATIVE_PRECONDITIONS_KEY',
    ':rewards'                  : 'REWARDS_KEY',
    ':disjunctive-preconditions': 'DISJUNCTIVE_PRECONDITIONS_KEY',
    ':conditional-effects'      : 'CONDITIONAL_EFFECTS_KEY',
    'when'                      : 'WHEN_KEY',
    ':constants'                : 'CONSTANTS_KEY'
}


def t_KEYWORD(t):
    #print('t_KEYWORD')
    r':?[a-zA-z_][a-zA-Z_0-9\-]*'
    t.type = reserved.get(t.value, 'NAME')
    return t


def t_NAME(t):
    #print('t_NAME')
    r'[a-zA-z_][a-zA-Z_0-9\-]*'
    #r'[a-zA-Z_0-9\-][a-zA-Z_0-9\-]*'
    return t


def t_VARIABLE(t):
    #print('t_VARIABLE')
    r'\?[a-zA-z_][a-zA-Z_0-9\-]*'
    #r'\?[a-zA-Z_0-9\-][a-zA-Z_0-9\-]*'
    return t


def t_PROBABILITY(t):
    #print('t_PROBABILITY')
    r'[0-1]\.\d+'
    t.value = float(t.value)
    return t

def t_REWARD_VALUE(t):
    #print('t_REWARD_VALUE')
    r'(-?[\d]+)' #need to change this so be + or - and not just btwn 0 and 1 #TODO
    t.value = float(t.value)
    return t

def t_newline(t):
    #print('t_newline')
    r'\n+'
    t.lineno += len(t.value)


def t_error(t):
    print("Error: illegal character '{0}'".format(t.value[0]))
    #traceback.print_stack() 
    t.lexer.skip(1)


# build the lexer
lex.lex()


def p_pddl(p):
    '''pddl : domain
            | problem'''
    if debug:
        print('p_pddl')
    p[0] = p[1]


def p_domain(p):
    '''domain : LPAREN DEFINE_KEY domain_def require_def types_def predicates_def constants_def constraints_def action_def_lst RPAREN
              | LPAREN DEFINE_KEY domain_def require_def types_def predicates_def constraints_def action_def_lst RPAREN
              | LPAREN DEFINE_KEY domain_def require_def types_def predicates_def action_def_lst RPAREN'''

    if debug:
        print('p_domain')
    if len(p) == 10:
        print('hello 10')
        p[0] = Domain(p[3], p[4], p[5], p[6], p[8],constraints=p[7])
    elif len(p) == 9:
        print('hello 9')
        p[0] = Domain(p[3], p[4], p[5], p[6], p[7])
    elif len(p) == 11:
        print('hello 11', p[6])
        p[0] = Domain(p[3], p[4], p[5], p[6], p[9],constraints=p[8],constants=p[7])


def p_problem(p):
    '''problem : LPAREN DEFINE_KEY problem_def domain_def objects_def init_def goal_def RPAREN'''
    if debug:
        print("p_problem")
    p[0] = Problem(p[3], p[4], p[5], p[6], p[7])


def p_domain_def(p):
    '''domain_def : LPAREN DOMAIN_KEY NAME RPAREN'''
    if debug:
        print('p_domain_def')
    p[0] = p[3]


def p_problem_def(p):
    '''problem_def : LPAREN PROBLEM_KEY NAME RPAREN'''
    if debug:
        print('p_problem_def')
    p[0] = p[3]


def p_objects_def(p):
    '''objects_def : LPAREN OBJECTS_KEY typed_constants_lst RPAREN'''
    if debug:
        print('p_objects_def')
    p[0] = p[3]

def p_constants_def(p):
    '''constants_def : LPAREN CONSTANTS_KEY typed_constants_lst RPAREN'''
    if debug:
        print('p_constants_def')
    p[0] = p[3]


def p_init_def(p):
    '''init_def : LPAREN INIT_KEY LPAREN AND_KEY ground_predicates_lst RPAREN RPAREN
                | LPAREN INIT_KEY ground_predicates_lst RPAREN'''
    if debug:
        print('p_init_def')
    if len(p) == 5:
        p[0] = p[3]
    elif len(p) == 8:
        p[0] = p[5]


def p_goal_def(p):
    '''goal_def : LPAREN GOAL_KEY LPAREN AND_KEY ground_predicates_lst RPAREN RPAREN'''
    if debug:
        print('p_goal_def')
    p[0] = p[5]


def p_require_def(p):
    '''require_def : LPAREN REQUIREMENTS_KEY require_key_lst RPAREN'''
    if debug:
        print('p_require_def')
    p[0] = p[3]


def p_require_key_lst(p):
    '''require_key_lst : require_key require_key_lst
                       | require_key'''
    if debug:
        print('p_require_key_lst')
    if len(p) == 2:
        p[0] = [p[1]]
    elif len(p) == 3:
        p[0] = [p[1]] + p[2]


def p_require_key(p):
    '''require_key : STRIPS_KEY
                   | EQUALITY_KEY
                   | TYPING_KEY
                   | PROBABILISTIC_EFFECTS_KEY
                   | NEGATIVE_PRECONDITIONS_KEY
                   | REWARDS_KEY
                   | DISJUNCTIVE_PRECONDITIONS_KEY
                   | CONDITIONAL_EFFECTS_KEY'''
    if debug:
        print('p_require_key')
    p[0] = str(p[1])


def p_types_def(p):
    '''types_def : LPAREN TYPES_KEY names_lst RPAREN'''
    if debug:
        print('p_types_def')
    p[0] = p[3]


def p_predicates_def(p):
    '''predicates_def : LPAREN PREDICATES_KEY predicate_def_lst RPAREN'''
    if debug:
        print('p_predicates_def')
    p[0] = p[3]


# def p_reward_def(p):
#     '''reward_def : LPAREN INCREASE REWARD PROBABILITY RPAREN'''

#     print('p_reward_def')
#     p[0] = p[3]

def p_predicate_def_lst(p):
    '''predicate_def_lst : predicate_def predicate_def_lst
                         | predicate_def'''
    if debug:
        print('p_predicate_def_lst')
    if len(p) == 2:
        p[0] = [p[1]]
    elif len(p) == 3:
        p[0] = [p[1]] + p[2]


def p_predicate_def(p):
    '''predicate_def : LPAREN NAME typed_variables_lst RPAREN
                     | LPAREN NAME RPAREN'''

    if debug:
        print("p_predicate_def")                
        print(p[0],p[1],p[2],p[3])
    if p[2] == 'increase':
        if debug:
            print("increase founds")
            print(p[2])

    if debug:
        print(p)
    if len(p) == 4:
        if debug:
            print("Length 4")
            print(p[0],p[1],p[2],p[3])
        p[0] = Predicate(p[2])
    elif len(p) == 5:
        if debug:
            print("Length 5")
            print(p[0],p[1],p[2],p[3],p[4])
        p[0] = Predicate(p[2], p[3])


def p_constraint(p):
    '''constraint : LPAREN FORALL_KEY LPAREN typed_variables_lst RPAREN LPAREN ATMOSTONCE_KEY literal RPAREN RPAREN
              '''
    if debug:
        print('p_constraint')
    if len(p) == 11:
        #p[0] = (p[4][0].__str__(),p[8],p[7]) # basic version
        p[0] = Constraint(p[7],p[8],p[4]) # constraint name, literal, typed_variables_lst
        if debug:
            print(p[0])
    if debug:
        print('p_constraint finished')

def p_constraints_def(p):
    '''constraints_def : LPAREN CONSTRAINTS_KEY LPAREN AND_KEY constraints_lst RPAREN RPAREN
                   | LPAREN CONSTRAINTS_KEY constraint RPAREN'''
    if len(p) == 5:
        p[0] = [p[3]]
    elif len(p) == 8:
        p[0] = p[5]


def p_constraints_lst(p):
    '''constraints_lst : constraint constraints_lst
                   | constraint'''
    if len(p) == 2:
        p[0] = [p[1]]
    elif len(p) == 3:
        p[0] = [p[1]] + p[2]

def p_action_def_lst(p):
    '''action_def_lst : action_def action_def_lst
                      | action_def'''
    if debug:
        print('p_action_def_lst')
    if len(p) == 2:
        p[0] = [p[1]]
    elif len(p) == 3:
        p[0] = [p[1]] + p[2]


def p_action_def(p):
    '''action_def : LPAREN ACTION_KEY NAME parameters_def action_def_body RPAREN'''
    if debug:
        print('p_action_def')
    p[0] = Action(p[3], p[4], p[5][0], p[5][1])


def p_parameters_def(p):
    '''parameters_def : PARAMETERS_KEY LPAREN typed_variables_lst RPAREN
                      | PARAMETERS_KEY LPAREN RPAREN'''
    if debug:
        print('p_parameters_def')
    if len(p) == 4:
        p[0] = []
    elif len(p) == 5:
        p[0] = p[3]


def p_action_def_body(p):
    '''action_def_body : precond_def effects_def'''
    if debug:
        print('p_action_def_body')
    p[0] = (p[1], p[2])


def p_precond_def(p):
    '''precond_def : PRECONDITION_KEY LPAREN AND_KEY literals_lst RPAREN
                   | PRECONDITION_KEY LPAREN OR_KEY literals_lst RPAREN
                   | PRECONDITION_KEY literal'''
    if debug:
        print('p_precond_def')
    if len(p) == 3:
        p[0] = [p[2]]
    elif len(p) == 6:
        if p[3] == 'and':
            p[0] = p[4]
        elif p[3] == 'or':
            p[0] = ['or', p[4]]



def p_effects_def(p):
    '''effects_def : EFFECT_KEY LPAREN AND_KEY effects_lst RPAREN
                   | EFFECT_KEY effect'''

    if debug:
        print('p_effects_def')
    if len(p) == 3:
        p[0] = [p[2]]
    elif len(p) == 6:
        p[0] = p[4]


def p_effects_lst(p):
    '''effects_lst : effect effects_lst
                   | effect'''
    if debug:
        print('p_effects_lst')
    if len(p) == 2:
        p[0] = [p[1]]
    elif len(p) == 3:
        p[0] = [p[1]] + p[2]

def p_probability_lst(p):
    '''probability_lst : PROBABILITY literal
                       | PROBABILITY literal probability_lst
                       | PROBABILITY LPAREN AND_KEY literals_lst RPAREN
                       | PROBABILITY LPAREN AND_KEY literals_lst RPAREN probability_lst'''
    if len(p) == 3:
        p[0] = (p[1], [p[2]])
    elif len(p) == 4:
        p[0] = (p[1], [p[2]]) + p[3]
    elif len(p) == 6:
        p[0] = (p[1],p[4])
    elif len(p) == 7:
        p[0] = (p[1], p[4]) + p[6]


def p_effect(p):
    '''effect : literal
              | LPAREN AND_KEY literals_lst RPAREN
              | LPAREN PROBABILISTIC_KEY probability_lst RPAREN
              | LPAREN WHEN_KEY literal literal RPAREN
              | LPAREN WHEN_KEY literal LPAREN PROBABILISTIC_KEY probability_lst RPAREN RPAREN
              | LPAREN WHEN_KEY LPAREN AND_KEY literals_lst RPAREN LPAREN AND_KEY literals_lst RPAREN RPAREN
              | LPAREN WHEN_KEY LPAREN AND_KEY literals_lst RPAREN LPAREN PROBABILISTIC_KEY probability_lst RPAREN RPAREN'''

    if debug:
        print('p_effect')

    if len(p) == 2:
        if debug:
            print(p[1])
        p[0] = (1.0, [p[1]]) # 1.0 represents 100% probability, since a prob isn't specified 
    elif len(p) == 5: # Prints reward, value instead of 1.0, value
        if p[2] != 'and':
            p[0] = p[3]
        else:
            p[0] = (1.0, p[3])
    elif len(p) == 6:
        if p[2] == 'when':
            p[0] = ['when', p[3], 'do', p[4]]
    elif len(p) == 9:
        if p[2] == 'when': 
            p[0] = ['when', p[3], 'do', (p[5], p[6])]
    elif len(p) == 12:
        if p[8] == 'probabilistic':
            p[0] = ['when', p[5], 'do', p[9]]
        else:
            p[0] = ['when', p[5], 'do', p[9]]
    if debug:
        print('p_effect finished')

def p_literals_lst(p):
    '''literals_lst : literal literals_lst
                    | literal'''
    if debug:
        print('p_literals_lst')
    if len(p) == 2:
        p[0] = [p[1]]
    elif len(p) == 3:
        p[0] = [p[1]] + p[2]


def p_literal(p):
    '''literal : LPAREN NOT_KEY predicate RPAREN
               | predicate
               | LPAREN INCREASE REWARD REWARD_VALUE RPAREN
               | ground_predicate
               | LPAREN NOT_KEY ground_predicate RPAREN'''
    if debug:
        print("In p literal")
        print(p[0],p[1])
    if len(p) == 2:
        p[0] = Literal.positive(p[1])
    elif len(p) == 5:
        p[0] = Literal.negative(p[3])
    elif len(p) == 6:
        p[0] = ('reward',p[4])


def p_ground_predicates_lst(p):
    '''ground_predicates_lst : ground_predicate ground_predicates_lst
                             | ground_predicate'''
    if debug:
        print('p_ground_predicates_lst')
    if len(p) == 2:
        p[0] = [p[1]]
    elif len(p) == 3:
        p[0] = [p[1]] + p[2]

def p_predicate(p):
    '''predicate : LPAREN NAME variables_lst RPAREN
                 | LPAREN EQUALS VARIABLE VARIABLE RPAREN
                 | LPAREN NAME RPAREN'''

    if debug:
        print("p_predicate")
        print(p[0],p[1],p[2],p[3])
    if p[2] == 'increase':
        if debug:
            print("increase founds")
            print(p[2])

    if len(p) == 4:
        p[0] = Predicate(p[2])
    elif len(p) == 5:
        p[0] = Predicate(p[2], p[3])

    elif len(p) == 6:
        if debug:
            print("Whatever I want pt 2")
        p[0] = Predicate('=', [p[3], p[4]])


def p_reward(p):
    '''reward : LPAREN INCREASE REWARD REWARD_VALUE RPAREN'''

    if debug:
        print("p_reward")    
    p[0] = Reward(p[4]) 

    

def p_ground_predicate(p):
    '''ground_predicate : LPAREN NAME constants_lst RPAREN
                        | LPAREN NAME RPAREN'''
    if debug:
        print('p_ground_predicate')
    if len(p) == 4:
        p[0] = Predicate(p[2])
    elif len(p) == 5:
        p[0] = Predicate(p[2], p[3])


def p_typed_constants_lst(p):
    '''typed_constants_lst : constants_lst HYPHEN type typed_constants_lst
                           | constants_lst HYPHEN type'''
    if debug:
        print('p_typed_constants_lst')
    if len(p) == 4:
        p[0] = [ Term.constant(value, p[3]) for value in p[1] ]
    elif len(p) == 5:
        p[0] = [ Term.constant(value, p[3]) for value in p[1] ] + p[4]


def p_typed_variables_lst(p):
    '''typed_variables_lst : variables_lst HYPHEN type typed_variables_lst
                           | variables_lst HYPHEN type'''
    if debug:
        print('p_typed_variables_lst')
    if len(p) == 4:
        p[0] = [ Term.variable(name, p[3]) for name in p[1] ]
    elif len(p) == 5:
        p[0] = [ Term.variable(name, p[3]) for name in p[1] ] + p[4]


def p_constants_lst(p):
    '''constants_lst : constant constants_lst
                     | constant'''
    if debug:
        print('p_constants_lst')
    if len(p) == 2:
        p[0] = [ Term.constant(p[1]) ]
    elif len(p) == 3:
        p[0] = [ Term.constant(p[1]) ] + p[2]


def p_variables_lst(p):
    '''variables_lst : VARIABLE variables_lst
                     | VARIABLE'''
    if debug:
        print('p_variables_lst')
    if len(p) == 2:
        p[0] = [p[1]]
    elif len(p) == 3:
        p[0] = [p[1]] + p[2]


def p_names_lst(p):
    '''names_lst : NAME names_lst
                 | NAME'''
    if debug:
        print('p_names_lst')
    if len(p) == 1:
        p[0] = []
    elif len(p) == 2:
        p[0] = [p[1]]
    elif len(p) == 3:
        p[0] = [p[1]] + p[2]


def p_type(p):
    '''type : NAME'''
    if debug:
        print('p_type')
    p[0] = p[1]


def p_constant(p):
    '''constant : NAME'''
    if debug:
        print('p_constant')
    p[0] = p[1]


def p_error(p):
    print("Error: syntax error when parsing '{}'".format(p))


# build parser
yacc.yacc()


class PDDLParser(object):

    @classmethod
    def parse(cls, filename):
        data = cls.__read_input(filename)
        return yacc.parse(data, debug=False)

    @classmethod
    def __read_input(cls, filename):
        with open(filename, 'r', encoding='utf-8') as file:
            data = ''
            for line in file:
                line = line.rstrip().lower()
                line = cls.__strip_comments(line)
                data += '\n' + line
        return data

    @classmethod
    def __strip_comments(cls, line):
        pos = line.find(';')
        if pos != -1:
            line = line[:pos]
        return line
