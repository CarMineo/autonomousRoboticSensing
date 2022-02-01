
Copyright (c) 2022, Carmelo Mineo
All rights reserved.

The main script (main.m) and all associated functions, demonstrate the work 
related to the paper titled: "Autonomous Robotic Sensing for Simultaneous
Geometric and Volumetric Inspection of Free-Form Parts", by C. Mineo,
D. Cerniglia and A. Poole.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution
  
* The names of the authors cannot be used to endorse or promote products 
  derived from this software, without specific prior written permission.

DISCLAIMER: 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

INSTRUCTIONS TO USE THE DEMONSTRATION SCRIPT:
In order to demonstrate the autonomous robotic inspection framework,
without a real inspection system, the present script allows generating a
random geometry. The generated geometry is displayed in the figure, to
inform the user and the function responsible to simulate sensor data. The
incremental exploration of the part geometry, through the proposed
framework, leads to the generation of a surface reconstruction mesh and
textured 3D maps of the sensor amplitude and sensor standoff. The little
variability of the amplitude and of the sensor standoff, despite of the
part surface curvature, are a demonstration of the good functionality of
the autonomous pose correction and navigation algorithms. The user can
control the type of surface to generate, through some key geometry
properties, as well as indicating the key input parameters for the
inspection (stepping angle for amplitude mapping, sensor noise amplitude,
target sensor standoff target resolution, angle for selection of second
sensor pose, and preferred initial rotation direction). Instead, the
initial inspection pose is randomly selected in this demonstration
script, in order to prove the robustness of the framework. A new random
shape and starting pose are generated for each execution of the script.
The authors of this work hope this demonstration script can help the full
understanding of the framework, which is fully described in the linked
scientific publication, and guide the adaptation of the framework
functions to deploy it to real applications. Do not hesitate to contact
the author, if you need any clarifications.
