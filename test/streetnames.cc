#include "baldr/streetnames.h"
#include "baldr/streetname.h"

#include <gtest/gtest.h>

#include <vector>

using namespace std;
using namespace valhalla::baldr;

namespace {

void TryListCtor(const std::vector<std::pair<std::string, bool>>& names) {
  StreetNames street_names(names);

  int x = 0;
  for (const auto& street_name : street_names) {
    EXPECT_EQ(names.at(x).first, street_name->value());
    EXPECT_EQ(names.at(x).second, street_name->is_route_number());
    ++x;
  }
}

TEST(Streetnames, TestListCtor) {
  TryListCtor({{"Main Street", false}});
  TryListCtor({{"Hershey Road", false}, {"PA 743 North", true}});
  TryListCtor({{"Unter den Linden", false}, {"B 2", true}, {"B 5", true}});
}

void TryFindCommonStreetNames(const StreetNames& lhs,
                              const StreetNames& rhs,
                              const StreetNames& expected) {
  std::unique_ptr<StreetNames> computed = lhs.FindCommonStreetNames(rhs);
  EXPECT_EQ(computed->ToString(), expected.ToString())
      << "Incorrect street names returned from FindCommonStreetNames";
}

StreetNames StreetNamesInit(const std::vector<std::pair<std::string, bool>>& names) {
  return StreetNames(names);
}

TEST(Streetnames, TestFindCommonStreetNames) {
  TryFindCommonStreetNames(StreetNamesInit({{"Hershey Road", false}, {"PA 743 North", true}}),
                           StreetNamesInit({{"Fishburn Road", false}, {"PA 743 North", true}}),
                           StreetNamesInit({{"PA 743 North", true}}));

  TryFindCommonStreetNames(StreetNamesInit({{"Hershey Road", false}, {"PA 743 North", true}}),
                           StreetNamesInit({{"Fishburn Road", false}, {"PA 743", true}}),
                           StreetNames());

  TryFindCommonStreetNames(StreetNamesInit({{"Capital Beltway", false},
                                            {"I 95 South", true},
                                            {"I 495 South", true}}),
                           StreetNamesInit({{"I 95 South", true}}),
                           StreetNamesInit({{"I 95 South", true}}));

  TryFindCommonStreetNames(StreetNamesInit(
                               {{"Unter den Linden", false}, {"B 2", true}, {"B 5", true}}),
                           StreetNamesInit({{"B 2", true}, {"B 5", true}}),
                           StreetNamesInit({{"B 2", true}, {"B 5", true}}));
}

void TryFindCommonBaseNames(const StreetNames& lhs,
                            const StreetNames& rhs,
                            const StreetNames& expected) {
  std::unique_ptr<StreetNames> computed = lhs.FindCommonBaseNames(rhs);
  EXPECT_EQ(computed->ToString(), expected.ToString())
      << "Incorrect street names returned from FindCommonBaseNames";
}

TEST(Streetnames, TestFindCommonBaseNames) {
  TryFindCommonBaseNames(StreetNamesInit({{"Hershey Road", false}, {"PA 743 North", true}}),
                         StreetNamesInit({{"Fishburn Road", false}, {"PA 743 North", true}}),
                         StreetNamesInit({{"PA 743 North", true}}));

  TryFindCommonBaseNames(StreetNamesInit({{"Unter den Linden", false}, {"B 2", true}, {"B 5", true}}),
                         StreetNamesInit({{"B 2", true}, {"B 5", true}}),
                         StreetNamesInit({{"B 2", true}, {"B 5", true}}));
}

void TryGetRouteNumbers(const StreetNames& street_names, const StreetNames& expected) {
  std::unique_ptr<StreetNames> computed = street_names.GetRouteNumbers();
  EXPECT_EQ(computed->ToString(), expected.ToString())
      << "Incorrect values returned from GetRouteNumbers";
}

TEST(Streetnames, TestGetRouteNumbers) {
  TryGetRouteNumbers(StreetNamesInit({{"Hershey Road", false}, {"PA 743 North", true}}),
                     StreetNamesInit({{"PA 743 North", true}}));

  TryGetRouteNumbers(StreetNamesInit({{"Unter den Linden", false}, {"B 2", true}, {"B 5", true}}),
                     StreetNamesInit({{"B 2", true}, {"B 5", true}}));

  TryGetRouteNumbers(StreetNamesInit({{"I 95 South", true}}),
                     StreetNamesInit({{"I 95 South", true}}));

  TryGetRouteNumbers(StreetNamesInit({{"Sheridan Circle", false}}), StreetNames());
}

void TryGetNonRouteNumbers(const StreetNames& street_names, const StreetNames& expected) {
  std::unique_ptr<StreetNames> computed = street_names.GetNonRouteNumbers();
  EXPECT_EQ(computed->ToString(), expected.ToString())
      << "Incorrect values returned from GetNonRouteNumbers";
}

TEST(Streetnames, TestGetNonRouteNumbers) {
  TryGetNonRouteNumbers(StreetNamesInit({{"Hershey Road", false}, {"PA 743 North", true}}),
                        StreetNamesInit({{"Hershey Road", false}}));

  TryGetNonRouteNumbers(StreetNamesInit({{"Unter den Linden", false}, {"B 2", true}, {"B 5", true}}),
                        StreetNamesInit({{"Unter den Linden", false}}));

  TryGetNonRouteNumbers(StreetNamesInit({{"I 95 South", true}}), StreetNames());

  TryGetNonRouteNumbers(StreetNamesInit({{"Sheridan Circle", false}}),
                        StreetNamesInit({{"Sheridan Circle", false}}));
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
