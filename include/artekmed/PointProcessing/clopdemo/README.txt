CLOP Source
-----------------------------------------------------------------------
Reference Implementation for
Preiner et al., Continuous Projection for Fast L1 Reconstruction, 
In Proceedings of ACM SIGGRAPH 2014
www.cg.tuwien.ac.at/research/publications/2014/preiner2014clop
-----------------------------------------------------------------------
(c) Reinhold Preiner, Vienna University of Technology, 2014
All rights reserved. This code is licensed under the New BSD License:
http://opensource.org/licenses/BSD-3-Clause
Contact: rp@cg.tuwien.ac.at
-----------------------------------------------------------------------



This demo contains a C++ CPU-only reference implementation of CLOP.

/src      
contains the pure source files. The implemenation is kept "header-only" for easy integration.

/vs2013   
provides a visual studio solution, containing a simple demo of CLOP. It loads the file "pointcloud.off", 
projects a subset of its points using CLOP, and writes the output to "pointcloud_projected.off". See the 
file main.cpp for the used parameters.



Version 0.9
-----------------------------------------------------------------------
Not contained in this version is the normal projection described in the paper. 
Although the class cp::Mixture provides means for computing the spherical normal distributions 
for the mixture, the robust normal alignment is not yet implemented.




Terms of usage
-----------------------------------------------------------------------
If this code is used in your publication, please cite the corresponding paper:

"Reinhold Preiner, Oliver Mattausch, Murat Arikan, Renato Pajarola, Michael Wimmer
Continuous Projection for Fast L1 Reconstruction
ACM Transactions on Graphics (Proc. of ACM SIGGRAPH 2014), 33(4):47:1-47:13, August 2014."

The corresponding BibTeX entry can be found here
http://www.cg.tuwien.ac.at/research/publications/2014/preiner2014clop/#BibTeX

Any feedback directed to the contact adress above is welcome!
