import interface_parser as parser
import template_instanstiator as instantiator

filename = "/home/duynguyen/git/enabling-anzu/perception/pybind_wrapper_test/wrap_example_py.h"
# filename = "/home/duynguyen/git/gtsam/gtsam.h"
with open(filename, "r") as f:
    content = f.read()
content

module = parser.Module.parseString(content)

print("Before:")
print(module.content[1].content[3].template)
print(module.content[1].content[3].name)

instantiator.instantiate_namespace_inplace(module)
print("After:")
print(module.content[1].content[3].template)
print(module.content[1].content[3].name)

inst = module.content[1].content[3]
inst.methods
