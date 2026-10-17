#include <gtest/gtest.h>

#include "graph_msf/core/TimeGraphKeyBuffer.h"

TEST(TimeGraphKeyBuffer, HandlesMeasurementsOutsideBuffer) {
  graph_msf::TimeGraphKeyBuffer buffer(3, 0);
  double stamp = 0.0;
  gtsam::Key key = 0;
  EXPECT_FALSE(buffer.getClosestKeyAndTimestamp(stamp, key, "test", 0.1, 1.0));
  buffer.addToBuffer(1.0, 10);
  buffer.addToBuffer(2.0, 20);
  EXPECT_TRUE(buffer.getClosestKeyAndTimestamp(stamp, key, "test", 0.1, 1.05));
  EXPECT_EQ(key, 10u);
  EXPECT_TRUE(buffer.getClosestKeyAndTimestamp(stamp, key, "test", 0.1, 2.05));
  EXPECT_EQ(key, 20u);
  EXPECT_FALSE(buffer.getClosestKeyAndTimestamp(stamp, key, "test", 0.1, 0.0));
  EXPECT_FALSE(buffer.getClosestKeyAndTimestamp(stamp, key, "test", 0.1, 3.0));
}

TEST(TimeGraphKeyBuffer, EvictsMatchingKeyAndReplacesDuplicateTimestamp) {
  graph_msf::TimeGraphKeyBuffer buffer(2, 0);
  buffer.addToBuffer(1.0, 10);
  buffer.addToBuffer(2.0, 20);
  buffer.addToBuffer(2.0, 21);
  EXPECT_EQ(buffer.getKeyToTimeBuffer().count(20), 0u);
  buffer.addToBuffer(3.0, 30);
  EXPECT_EQ(buffer.getTimeToKeyBuffer().count(1.0), 0u);
  EXPECT_EQ(buffer.getKeyToTimeBuffer().count(10), 0u);
  EXPECT_EQ(buffer.getKeyToTimeBuffer().at(21), 2.0);
}
