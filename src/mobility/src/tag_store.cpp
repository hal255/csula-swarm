#include "tag_store.h"

TagStore::TagStore()
{
  tags_collected_ = 0;
}

void TagStore::push(int id, TagObject tag)
{
  if (!tags_[id].isTagDetected())
  {
    tags_[id] = tag;
  }
}

TagObject TagStore::pop(geometry_msgs::Pose2D pos)
{
  int easiest_index = 0;
  for (int i = 0; i < sizeof(tags_); i++)
  {
    if (tags_[i].isTagDetected() && !tags_[i].isTagCollected() && tags_[i].isAvailable())
    {
      double easiest_linear_distance = tags_[easiest_index].getLinearDistance();
      double easiest_angular_distance = tags_[easiest_index].getAngularDistance();
      double current_linear_distance = tags_[i].getLinearDistance();
      double current_angular_distance = tags_[i].getAngularDistance();
      double weighted_easist_distance = easiest_linear_distance + easiest_angular_distance / M_PI / 2 * 5;
      double weighted_current_distance = current_linear_distance + current_angular_distance / M_PI / 2 * 5;
      // if (weighted_current_distance < weighted_easist_distance)
      // {
        //setUnavailable(i);
        return tags_[i];
        //easiest_index = i;
     // }
    }
  }
  tags_[easiest_index].setUnavailable();
  return tags_[easiest_index];
}

void TagStore::collected(int id)
{
  tags_[id].setTagCollected();
  tags_collected_++;
}

void TagStore::setUnavailable(int id)
{
  tags_[id].setUnavailable();
}

bool TagStore::tagAvailable()
{
  for(int i = 0; i < 256; ++i)
  {
    if (tags_[i].isTagDetected() && !tags_[i].isTagCollected() && tags_[i].isAvailable())
    {
      return true;
    }
  }
  return false;
}

bool TagStore::isTagAvailable(int id)
{
  return !tags_[id].isTagDetected() || tags_[id].isAvailable();
}

int TagStore::tagsCollected()
{
  return tags_collected_;
}