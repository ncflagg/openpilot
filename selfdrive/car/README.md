This is a started project to see if I can mimic the old op-vsr code without modifying the steering ratio which, should resolve the calibration issues (green projected path curvature behavior is altered) I randomly see in 0.5.6 - 0.5.8.

calm-center currently doesn't take all of the learned angle offest into account yet.

I'm realizing that the current code dampens the reported angle only around static center; it needs to be a moving window centered around the current wheel position and if a 'corner' is not being navigated.
