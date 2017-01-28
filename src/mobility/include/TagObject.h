#ifndef TAGOBJECT_H
#define TAGOBJECT_H

class TagObject
{
public:
  TagObject();

  void initi(double x, double y);

  // Accessors
  double getX();
  double getY();
  double getLinearDistance();
  double getAngularDistance();
  //int getId()
  bool isTagCollected();
  void setTagCollected();
  bool isTagDetected();
  void setTagDetected();
  bool isAvailable();
  void setAvailable();
  void setUnavailable();

private:
  static const double PI = 3.141592653589793;
  int tag_id_;
  double x_;
  double y_;
  double linear_distance_;
  double angular_distance_;
  bool tag_collected_;
  bool tag_detected_;
  bool tag_available_;
  // Private member functions
  void setDistance();
  void onAxis();
};

#endif /* TAGOBJECT_H */