# This file is an addition to the pypddl-parser

# Emily Scheide (October 2021)


from term import Term


class Reward(object):

    def __init__(self, value):
        self._value = value

    @property
    def value(self):
        return self._value

    def __str__(self):
        return str(self._value)