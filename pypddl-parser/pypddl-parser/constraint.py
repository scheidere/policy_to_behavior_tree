# This file is an addition to the pypddl-parser

# Emily Scheide (January 2022)


from term import Term


class Constraint(object):

    def __init__(self, name, literal, params):
        self._name    = name
        self._literal = literal
        self._params  = params # typed_variables_list

    @property
    def name(self):
        return self._name

    def literal(self):
    	return self._literal

    @property
    def params(self):
        return self._params[:]

    def __str__(self):
        constraint_str  = '{0}[{1}({2})]'.format(self._name, self._literal._predicate._name, ', '.join(map(str, self._params)))
        return constraint_str