
# HW4: Applying Trajectory Optimization 
## Due date: Friday, April 15

In this homework you'll implement a few of the methods we've talked about in class that extend the traditional formulation of trajectory optimization.
1. Design a barrell roll maneuver for an airplane by modifying iLQR to work with quaternions
2. Design a walking gain for a simple bipedal walker using Hybrid Trajectory Optimization (using Ipopt)
3. Refine an optimized solution to compensate for modelling errors using Iterative Learning Control (ILC)


## Grading Policy
See [this document](https://github.com/Optimal-Control-16-745/JuliaIntro/blob/main/docs/Submission%20Instructions.md) for details on our grading policy. 
Do NOT attempt to hack the autograder. We will be looking at all of your 
solutions and any attempt to modify the autograder in any way will result in
heavy penalties. It's there to help make your lives easier, don't abuse it!

## Getting Started
All the homeworks are distributed as Jupyter notebooks. Follow these instructions to get everything set up.

1. Install Julia. Download v1.6.5 from the [Julia website](https://julialang.org/downloads/). Extract the binaries onto a stable location on your computer, and add the `/bin` folder to your system path.
2. Clone this repo, and open a terminal in the root directory
2. Start a Julia REPL in your terminal using `julia`. This should work as long as the binaries are on your system path.
3. Install the [IJulia](https://github.com/JuliaLang/IJulia.jl) using the Julia package manager. In the REPL, enter the package manager using `]`, then enter `add IJulia` to add it to your system path.
4. In the REPL (hit backspace to exit the package manager), enter `using IJulia`
5. Launch the notebook using `notebook()` or `jupyterlab()`

Check out Kevin's [video walkthrough](https://www.youtube.com/watch?v=I2SC1Mp3Hxs&feature=youtu.be) for HW0 for more details.


## Tips for Success
These homeworks can be pretty challenging, which is why we provide 2 weeks to complete them. Here are some tips for success:
1. Start early
2. Commit and push changes often. Don't be afraid to use git branches. This is 
super helpful if you ever need to go back to a previous version because you broke something.
3. Get with a study group!
4. Keep your code clean. Avoid global variables. If you get odd behavior, consider restarting your kernel.
5. Come to office hours


## Running the Autograder
From a terminal in the root of your repository, run the following command:
```shell
julia --project=. test/runtests.jl
```
You can select which questions to grade by passing them in as arguments. For example, to 
grade questions 1 and 3, use:
```shell
julia --project=. test/runtests.jl 1 3
```
We will use code very similar to the autograder included in the student repos to 
grade your homeworks, so if your code passes the autograder locally on your computer, you 
should be good!


## Questions / Issues
If you have questions as you work through the homework, please post to the 
`hw4` folder on Pizza. 

## Adding the Upstream Repo
We may release changes to the homework periodically if errors or bugs are found. Follow these instructions for linking your repo to the original template and pulling changes. It's always a good idea to branch your code before pulling from the upstream repo in case something goes wrong or the merge is particularly nasty. 