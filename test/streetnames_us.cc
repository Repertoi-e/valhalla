#include "baldr/streetname_us.h"
#include "baldr/streetnames_us.h"

#include <gtest/gtest.h>

#include <vector>

using namespace std;
using namespace valhalla::baldr;

namespace {

void TryListCtor(const std::vector<std::pair<std::string, bool>>& names) {
  StreetNamesUs street_names(names);

  int x = 0;
  for (const auto& street_name : street_names) {
    EXPECT_EQ(names.at(x).first, street_name->value());
    EXPECT_EQ(names.at(x).second, street_name->is_route_number());
    ++x;
  }
}

TEST(StreetnamesUs, TestListCtor) {
  TryListCtor({{"Main Street", false}});
  TryListCtor({{"Hershey Road", false}, {"PA 743 North", true}});
}

void TryFindCommonStreetNames(const StreetNamesUs& lhs,
                              const StreetNamesUs& rhs,
                              const StreetNamesUs& expected) {
  std::unique_ptr<StreetNames> computed = lhs.FindCommonStreetNames(rhs);
  EXPECT_EQ(computed->ToString(), expected.ToString()) << "FindCommonStreetNames";
}

StreetNamesUs StreetNamesUsInit(const std::vector<std::pair<std::string, bool>>& names) { return StreetNamesUs(names); }

TEST(StreetnamesUs, TestFindCommonStreetNames) {
  TryFindCommonStreetNames(StreetNamesUsInit({{"Hershey Road", false}, {"PA 743 North", true}}),
                           StreetNamesUsInit({{"Fishburn Road", false}, {"PA 743 North", true}}),
                           StreetNamesUsInit({{"PA 743 North", true}}));

  TryFindCommonStreetNames(StreetNamesUsInit({{"Hershey Road", false}, {"PA 743 North", true}}),
                           StreetNamesUsInit({{"Fishburn Road", false}, {"PA 743", true}}),
                           StreetNamesUs());

  TryFindCommonStreetNames(StreetNamesUsInit({{"Capital Beltway", false},
                                          {"I 95 South", true},
                                          {"I 495 South", true}}),
                           StreetNamesUsInit({{"I 95 South", true}}),
                           StreetNamesUsInit({{"I 95 South", true}}));
}

void TryFindCommonBaseNames(const StreetNamesUs& lhs,
                            const StreetNamesUs& rhs,
                            const StreetNamesUs& expected) {
  std::unique_ptr<StreetNames> computed = lhs.FindCommonBaseNames(rhs);
  EXPECT_EQ(computed->ToString(), expected.ToString()) << "FindCommonBaseNames";
}

TEST(StreetnamesUs, TestFindCommonBaseNames) {
  TryFindCommonBaseNames(StreetNamesUsInit({{"Hershey Road", false}, {"PA 743 North", true}}),
                         StreetNamesUsInit({{"Fishburn Road", false}, {"PA 743 North", true}}),
                         StreetNamesUsInit({{"PA 743 North", true}}));

  TryFindCommonBaseNames(StreetNamesUsInit({{"Hershey Road", false}, {"PA 743 North", true}}),
                         StreetNamesUsInit({{"Fishburn Road", false}, {"PA 743", true}}),
                         StreetNamesUsInit({{"PA 743 North", true}}));

  TryFindCommonBaseNames(StreetNamesUsInit({{"Hershey Road", false}, {"PA 743", true}}),
                         StreetNamesUsInit({{"Fishburn Road", false}, {"PA 743 North", true}}),
                         StreetNamesUsInit({{"PA 743 North", true}}));

  TryFindCommonBaseNames(StreetNamesUsInit({{"Hershey Road", false}, {"PA 743", true}}),
                         StreetNamesUsInit({{"Fishburn Road", false}, {"PA 743", true}}),
                         StreetNamesUsInit({{"PA 743", true}}));

  TryFindCommonBaseNames(StreetNamesUsInit({{"Capital Beltway", false},
                                        {"I 95 South", true},
                                        {"I 495 South", true}}),
                         StreetNamesUsInit({{"I 95 South", true}}),
                         StreetNamesUsInit({{"I 95 South", true}}));
}

void TryGetRouteNumbers(const StreetNamesUs& street_names, const StreetNamesUs& expected) {
  std::unique_ptr<StreetNames> computed = street_names.GetRouteNumbers();
  EXPECT_EQ(computed->ToString(), expected.ToString()) << "GetRouteNumbers";
}

TEST(StreetnamesUs, TestGetRouteNumbers) {
  TryGetRouteNumbers(StreetNamesUsInit({{"Hershey Road", false}, {"PA 743 North", true}}),
                     StreetNamesUsInit({{"PA 743 North", true}}));

  TryGetRouteNumbers(StreetNamesUsInit({{"Unter den Linden", false}, {"B 2", true}, {"B 5", true}}),
                     StreetNamesUsInit({{"B 2", true}, {"B 5", true}}));

  TryGetRouteNumbers(StreetNamesUsInit({{"I 95 South", true}}), StreetNamesUsInit({{"I 95 South", true}}));

  TryGetRouteNumbers(StreetNamesUsInit({{"Sheridan Circle", false}}), StreetNamesUs());

  TryGetRouteNumbers(StreetNamesUsInit(
                         {{"Capital Beltway", false}, {"I 95 South", true}, {"I 495 South", true}}),
                     StreetNamesUsInit({{"I 95 South", true}, {"I 495 South", true}}));
}

void TryGetNonRouteNumbers(const StreetNamesUs& street_names, const StreetNamesUs& expected) {
  std::unique_ptr<StreetNames> computed = street_names.GetNonRouteNumbers();
  EXPECT_EQ(computed->ToString(), expected.ToString()) << "GetNonRouteNumbers";
}

TEST(StreetnamesUs, TestGetNonRouteNumbers) {
  TryGetNonRouteNumbers(StreetNamesUsInit({{"Hershey Road", false}, {"PA 743 North", true}}),
                        StreetNamesUsInit({{"Hershey Road", false}}));

  TryGetNonRouteNumbers(StreetNamesUsInit({{"Unter den Linden", false}, {"B 2", true}, {"B 5", true}}),
                        StreetNamesUsInit({{"Unter den Linden", false}}));

  TryGetNonRouteNumbers(StreetNamesUsInit({{"I 95 South", true}}), StreetNamesUs());

  TryGetNonRouteNumbers(StreetNamesUsInit({{"Sheridan Circle", false}}),
                        StreetNamesUsInit({{"Sheridan Circle", false}}));

  TryGetNonRouteNumbers(StreetNamesUsInit({{"Capital Beltway", false},
                                       {"I 95 South", true},
                                       {"I 495 South", true}}),
                        StreetNamesUsInit({{"Capital Beltway", false}}));
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
