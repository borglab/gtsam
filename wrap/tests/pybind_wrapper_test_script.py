import pybind_wrapper_test as pwt

p2 = pwt.sub.Point2(10, 20)
print(p2.sum())
print(p2.func_with_default_args(10))
print(p2.func_with_default_args(10, 10))
p2.print_("test print")

p22 = pwt.sub.Point2(10)
print(p22.y())

p3 = pwt.Point3(10, 20, 30)
print(p3.sum())
print(p3.x(to_add=100))

print(pwt.global_func_on_base(p2))
print(pwt.global_func_on_base(p3))

# Test template class.
# Construct with POINT
template_p2 = pwt.TemplatePoint2(p2)
print(template_p2.overload())

template_p3 = pwt.TemplatePoint3(p3)
print(template_p3.overload())
print(template_p3.overload(p3))

# Construct with This
template_p3_copy = pwt.TemplatePoint3(template_p3)
print(template_p3_copy.overload())
print(template_p3.overload(template_p3_copy))

# Function of template type.
ret_p3 = template_p3.method_on_template_type(p3)
print(ret_p3.z())

# Function of This class type.
this = template_p3.method_on_this(p3)
print(this.method_on_template_type(p3).sum())

# Static function.
another_this = pwt.TemplatePoint2.static_method(other=template_p2, dummy=0.0)

# Template function of other POINT type.
print(another_this.template_methodPoint3(p3))
print(another_this.template_methodPoint2(p2))

# Typedef template instantiation.
inst = pwt.Template2Point2Point3Instantiation(p2, p3)
inst.property_t1 = pwt.sub.Point2(100)
print("inst overload: ", inst.sum_x())
print(inst.sum_x(p2))
print(inst.sum_x(p3))
print(inst.sum_x(p2, p3))
print(inst.property_t1.sum())

# Properties
p4 = pwt.sub2.Point4(p2, 30, 40)
print(p4.p.sum())
print(p4.sum())
p4.z = 40
print(p4.sum())
