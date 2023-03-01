#include <CppUnitLite/TestHarness.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

#include "gtsam/base/serializationTestHelpers.h"
#include "gtsam/base/std_optional_serialization.h"

using namespace gtsam;

// A test to check that the serialization of std::optional works with boost
TEST(StdOptionalSerialization, SerializeIntOptional) {
  // Create an optional
  std::optional<int> opt(42);

  // Serialize it
  std::stringstream ss;
  boost::archive::text_oarchive oa(ss);
  oa << opt;

  // Deserialize it
  std::optional<int> opt2;
  boost::archive::text_iarchive ia(ss);
  ia >> opt2;

  // Check that it worked
  EXPECT(opt2.has_value());
  EXPECT(*opt2 == 42);
}

// A test to check if we serialize an empty optional, it is deserialized as an
// empty optional
TEST(StdOptionalSerialization, SerializeEmptyOptional) {
  // Create an optional
  std::optional<int> opt;

  // Serialize it
  std::stringstream ss;
  boost::archive::text_oarchive oa(ss);
  oa << opt;

  // Deserialize it
  std::optional<int> opt2 = 43;
  boost::archive::text_iarchive ia(ss);
  ia >> opt2;

  // Check that it worked
  EXPECT(!opt2.has_value());
}

/* ************************************************************************* */
// Implement the equals trait for TestOptionalStruct
namespace gtsam {
// A struct which contains an optional
class TestOptionalStruct {
public:
  std::optional<int> opt;
  TestOptionalStruct() = default;
  TestOptionalStruct(const int& opt)
      : opt(opt) {}
  // A copy constructor is needed for serialization
  TestOptionalStruct(const TestOptionalStruct& other) = default;
  bool operator==(const TestOptionalStruct& other) const {
    // check the values are equal
    return *opt == *other.opt;
  }
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(opt);
  }
};

template <>
struct traits<TestOptionalStruct> {
  static bool Equals(const TestOptionalStruct& q1, const TestOptionalStruct& q2, double) {
    // if both have values then compare the values
    if (q1.opt.has_value() && q2.opt.has_value()) {
      return *q1.opt == *q2.opt;
    }
    // otherwise check that both are empty
    return q1.opt == q2.opt;
  }

  static void Print(const TestOptionalStruct& m, const std::string& s = "") {
    /* std::cout << s << "opt: " << m.opt << std::endl; */
    if (m.opt.has_value()) {
      std::cout << s << "opt: " << *m.opt << std::endl;
    } else {
      std::cout << s << "opt: empty" << std::endl;
    }
  }
};
}  // namespace gtsam

// Test for a struct with an initialized optional
TEST(StdOptionalSerialization, SerializTestOptionalStruct) {
  TestOptionalStruct optStruct(10);
  EXPECT(serializationTestHelpers::equalsObj(optStruct));
  EXPECT(serializationTestHelpers::equalsXML(optStruct));
  EXPECT(serializationTestHelpers::equalsBinary(optStruct));
}

// Test for a struct with an uninitialized optional
TEST(StdOptionalSerialization, SerializTestOptionalStructUninitialized) {
  TestOptionalStruct optStruct;
  EXPECT(serializationTestHelpers::equalsObj(optStruct));
  EXPECT(serializationTestHelpers::equalsXML(optStruct));
  EXPECT(serializationTestHelpers::equalsBinary(optStruct));
}

// Test for serialization of std::optional<TestOptionalStruct> 
TEST(StdOptionalSerialization, SerializTestOptionalStructPointer) {
  // Create an optional
  std::optional<TestOptionalStruct> opt(TestOptionalStruct(42));

  // Serialize it
  std::stringstream ss;
  boost::archive::text_oarchive oa(ss);
  oa << opt;

  // Deserialize it
  std::optional<TestOptionalStruct> opt2;
  boost::archive::text_iarchive ia(ss);
  ia >> opt2;

  // Check that it worked
  EXPECT(opt2.has_value());
  EXPECT(*opt2 == TestOptionalStruct(42));
}

// Test for serialization of std::optional<TestOptionalStruct*>
TEST(StdOptionalSerialization, SerializTestOptionalStructPointerPointer) {
  // Create an optional
  std::optional<TestOptionalStruct*> opt(new TestOptionalStruct(42));

  // Serialize it
  std::stringstream ss;
  boost::archive::text_oarchive oa(ss);
  oa << opt;

  // Deserialize it
  std::optional<TestOptionalStruct*> opt2;
  boost::archive::text_iarchive ia(ss);
  ia >> opt2;

  // Check that it worked
  EXPECT(opt2.has_value());
  EXPECT(**opt2 == TestOptionalStruct(42));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
