from collections import namedtuple

# TODO: Consider changing these to classes if I will translate from c++ to python documentaiton in class
'''Template for documentation of methods'''
method = namedtuple('method', [
    'return_type', 'definition', 'argsstring', 'name', 'reimplements',
    'params', 'brief_description', 'detailed_description', 'inbody_description'
])

'''Template for documentation of parameters'''
param = namedtuple('param', 'name type default_value')
