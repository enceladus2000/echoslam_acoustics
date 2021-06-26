# Experiments

This folder contains acoustic simulation experiments. 

## Results

`pipe_shoebox.py` creates a basic cuboidal 15m long 'pipe' with acoustically reflective surfaces at both ends. For the following impulse response, the source and mic where kept at x=13m.

![](https://github.com/enceladus2000/echoslam_acoustics/blob/master/media/pipe_shoebox_rir.png)

This matches with the Fig 2 in [this paper](https://www.researchgate.net/profile/Rob-Worley-2/publication/344071778_Acoustic_Echo-Localization_for_Pipe_Inspection_Robots/links/5f50d69c458515e96d26dd38/Acoustic-Echo-Localization-for-Pipe-Inspection-Robots.pdf). The few additional peaks at the beginning probably correspond to reflections from the side of the pipe.

## Under Development

### Pipebots
- `t_pipe.py`: Simulation is ready, need to analyse the results. 

### Other
- `kalman1D.py`: Basic Kalman implementation
