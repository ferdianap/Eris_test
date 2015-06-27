## This is the contents of a Semantic Memory,
## specifically about a single object only.
## Related pkg: eris
##
## Author: Ferdian Pratama

Header header
string label
string[] tag
sensor_msgs/Image image
int16[] color # Color structure Descriptor
int16[] texture # Edge Histogram Descriptor
#int16 link_amount
#string[] link_name # TODO: continue here implement how to link same object

#bool graspable
#bool manipulable
#float32[] shape # todo do not use this (gist)