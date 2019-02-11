from enabling_anzu.perception.tracking.wrap_example import (
    anzu,
    global_func_overloads,
)

p2 = anzu.sub.Point2(10, 20)
print p2.sum()
print p2.func_with_default_args(10)
print p2.func_with_default_args(10, 10)
p2.print_("test print")

p22 = anzu.sub.Point2(10)
print p22.y()

p3 = anzu.Point3(10, 20, 30)
print p3.sum()
print p3.x(to_add=100)

print global_func_overloads(p2)
print global_func_overloads(p3)

print anzu.global_func_on_base(p2)
print anzu.global_func_on_base(p3)

# Test template class.
# Construct with POINT
template_p2 = anzu.TemplatePoint2(p2)
print template_p2.overload()

template_p3 = anzu.TemplatePoint3(p3)
print template_p3.overload()
print template_p3.overload(p3)

# Construct with This
template_p3_copy = anzu.TemplatePoint3(template_p3)
print template_p3_copy.overload()
print template_p3.overload(template_p3_copy)

# Function of template type.
ret_p3 = template_p3.method_on_template_type(p3)
print ret_p3.z()

# Function of This class type.
this = template_p3.method_on_this(p3)
print this.method_on_template_type(p3).sum()

# Static function.
another_this = anzu.TemplatePoint2.static_method(other=template_p2, dummy=0.0)

# Template function of other POINT type.
print another_this.template_methodPoint3(p3)
print another_this.template_methodPoint2(p2)

# Typedef template instantiation.
inst = anzu.Template2Point2Point3Instantiation(p2, p3)
print "inst overload: ", inst.sum_x()
print inst.sum_x(p2)
print inst.sum_x(p3)
print inst.sum_x(p2, p3)
