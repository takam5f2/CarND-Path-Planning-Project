# Reflection
Self-Driving Car Engineer Nanodegree Program
   
### Video (Evidence for achievement of requirements.)
[Here](https://www.youtube.com/watch?v=Z73QFyxSDks) is video which shows successful path planning.
In this video, vehicle succeeded in running for 10 miles without any violation or collision.

My path planning algorithm can't correspond difficult situations.
For examle, my algorithm can't stop the ego vehicle immediately when other vehicle cut in the space between the ego and the preceding vehicle.
Besides, my algorithm let the ego vehicle change lane to left if left and right lane are empty. It is because the condition of changing left lane is evaluated earlier than the condition of right lane.


Except for difficult cases, my algorithm works well and achieve requirements provided by [project rubic](https://review.udacity.com/#!/rubrics/1971/view).



### Source Files


### Reflection about my algorithm of path planning.
I constructed path planning algorithm based on the walkthrough video.
Especially, trajectory planning based on spline interpolation is algorithm shown in the walkthrough video.


I mainly contributed to algorithm which decided which how the ego vehicle behaved.
My path planner has very simple algorithm. If there is preceding vehicle which is slower than the ego vehicle, the ego vehicle change its lane to left or right.

The ego vehicle tries to turn left lane if left lane is empty and the ego vehicle can't run at target velocity due to preceding vehicle. If left lane is occupied and right lane is empty, the ego vehicle tries to turn right lane. If both left and right lane are occupied, the ego vehicle give up changing lane and decelerate its own.

My path planner is consisted of simple logic because it is very easy to verify and validate my algorithm. At the beginning of this project, I tried to introduce FMS based on cost functions, but I notices that it is difficult to validate my path planner because each cost function has complicated relationship. It costs a lot of time to validate.
Considering the chacteristic of this project, solid logic is enough to complete this one. I means that running highway does not have much candidate of behavior, so simple path planner can tackle this problem.

Designing path planner for urban course, I should construct well-designed FMS and cost function.
