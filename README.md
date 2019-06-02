Prius calm-center is meant to be a replacement for the Variable Steering Ratio mod (VSR or op-vsr)
======

Trying to see if I can mimic the steering behavior of the old op-vsr code without modifying the steering ratio which, should resolve the calibration issues (green projected path curvature behavior is altered) I randomly see in 0.5.6 - 0.5.8. I'm trying to do the same thing in a more direct way since I think I better understand what VSR was really doing. Now, I'm just increasing the reported angle near steering center in order to calm down corrections.

I'm realizing, however, that the current code dampens the reported angle only around static center; it needs to be a moving window centered around the current wheel position and if a 'corner' is not being navigated.

calm-center currently doesn't take all of the learned angle offest into account yet.

(no toyota pedal)
