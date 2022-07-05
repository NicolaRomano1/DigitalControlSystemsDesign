# Nicola ROMANO         0622701549    N.ROMANO24@STUDENTI.UNISA.IT
# Antonio ROTONDO       0622701489    A.ROTONDO9@STUDENTI.UNISA.IT
# Catello SORRENTINO    06227001490   C.SORRENTINO61@STUDENTI.UNISA.IT

The STM project has been provided with all the necessary files to run,
however we also included the projects to generate the required files.


This is the list of things to do to create a project from scratch:

#1
Open the "OSQP_Matlab" file in the "MATLAB-project" folder and run it
to generate the matrices required in the project. Then it's needed to 
export the workspace which will be used in the next step.

#2
After saving the workspace file in the "OSQP-C-code-generation" folder,
open the file "CodeGeneration.py" and make sure that the ".mat" file called 
is the correct one, then run the script to generate the code.

#3
The script will generate a "code" folder. 
In ./src there will be an "osqp" folder, it has to be copied in the source 
folder of the STM project.
In ./inc there will be many include files, they have to be copied in the
include folder of the STM project.

The project is now ready to use OSQP.

The complete project can be found in the ST32-project folder.
