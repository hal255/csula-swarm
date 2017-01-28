#ifndef TAG_STORE_H
#define TAG_STORE_H

#include <geometry_msgs/Pose2D.h>

#include "TagObject.h"

class TagStore
{
public:
  TagStore();
  
  void push(int id, TagObject tag);
  TagObject pop(geometry_msgs::Pose2D pos);
  void collected(int id);
  void setUnavailable(int id);
  bool tagAvailable();
  bool isTagAvailable(int id);
  int tagsCollected();
  
private:
  //Array storing all the tags
  TagObject tags_[256];
  //Number of tags collected
  int tags_collected_;
};

#endif /* TAG_STORE_H */