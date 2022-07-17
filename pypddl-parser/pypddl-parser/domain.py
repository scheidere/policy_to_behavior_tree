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

class Domain(object):

    def __init__(self, name, requirements, types, predicates, operators,constraints=None,constants=None):
    #def __init__(self, name, requirements, types, predicates, operators):
        self._name = name
        self._requirements = requirements
        self._types = types
        if constants:  
            input('constants is NOT None')      
            self._constants = {}
            print('test2 consts', constants)
            for const in constants:
                self._constants[const.type] = self._constants.get(const.type, [])
                self._constants[const.type].append(str(const.value))
        else:
            input('constants is None')
            self._constants = constants # None
        # self._objects = {}
        # for obj in objects:
        #     self._objects[obj.type] = self._objects.get(obj.type, [])
        #     self._objects[obj.type].append(str(obj.value))
        self._predicates = predicates
        self._constraints = constraints # could be None
        self._operators = operators

    @property
    def name(self):
        return self._name

    @property
    def requirements(self):
        return self._requirements[:]

    @property
    def types(self):
        return self._types[:]

    # @property
    # def constants(self):
    #     return self._constants[:]

    # @property
    # def objects(self):
    #     return self._objects.copy()

    @property
    def constants(self):
        return self._constants.copy()

    @property
    def predicates(self):
        return self._predicates[:]

    @property
    def constraints(self):
        if self._constraints:
            return self._constraints[:]

    @property
    def operators(self):
        return self._operators[:]

    def __str__(self):
        domain_str  = '@ Domain: {0}\n'.format(self._name)
        domain_str += '>> requirements: {0}\n'.format(', '.join(self._requirements))
        domain_str += '>> types: {0}\n'.format(', '.join(self._types))
        domain_str += '>> predicates: {0}\n'.format(', '.join(map(str, self._predicates)))
        if self._constants:
            domain_str += '>> constants:\n'
            for type, constants in self._constants.items():
                domain_str += '{0} -> {1}\n'.format(type, ', '.join(sorted(constants)))
        if self._constraints:
            domain_str += '>> constraints: {0}\n'.format(', '.join(map(str, self._constraints)))
        #print(domain_str)
        domain_str += '>> operators:\n    {0}\n'.format(
            '\n    '.join(str(op).replace('\n', '\n    ') for op in self._operators))
        return domain_str
