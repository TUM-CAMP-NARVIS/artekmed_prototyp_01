//=============================================================================
// CLOP Source
//-----------------------------------------------------------------------------
// Reference Implementation for
// Preiner et al., Continuous Projection for Fast L1 Reconstruction, In 
// Proceedings of ACM SIGGRAPH 2014
// www.cg.tuwien.ac.at/research/publications/2014/preiner2014clop
//-----------------------------------------------------------------------------
// (c) Reinhold Preiner, Vienna University of Technology, 2014
// All rights reserved. This code is licensed under the New BSD License:
// http://opensource.org/licenses/BSD-3-Clause
// Contact: rp@cg.tuwien.ac.at
//=============================================================================


#pragma once

#include "mixture.hpp"
#include <string>

using namespace std;


namespace cp
{
	
	struct PlyIO
	{

		/// export mixture to ascii PLY file format string
		static void exportMixture(const Mixture& mixture, string& plyString)
		{
			stringstream ply;
			ply << "ply" << endl;
			ply << "format ascii 1.0" << endl;
			ply << "comment mixture 1.0" << endl;
			ply << "comment clop exported Gaussian mixture ply file" << endl;
			ply << "comment (c) Reinhold Preiner" << endl;

			ply << "element component " << mixture.size() << endl;
			ply << "property float x" << endl;
			ply << "property float y" << endl;
			ply << "property float z" << endl;
			ply << "property float covxx" << endl;
			ply << "property float covxy" << endl;
			ply << "property float covxz" << endl;
			ply << "property float covyy" << endl;
			ply << "property float covyz" << endl;
			ply << "property float covzz" << endl;
			ply << "property float weight" << endl;
			ply << "property float nvx" << endl;
			ply << "property float nvy" << endl;
			ply << "property float nvz" << endl;
			ply << "end_header" << endl;

			for (uint i = 0; i < mixture.size(); ++i)
			{
				const Component& c = mixture[i];
				ply << c.µ.x << "  ";
				ply << c.µ.y << "  ";
				ply << c.µ.z << "  ";
				ply << c.cov.e00 << "  ";
				ply << c.cov.e01 << "  ";
				ply << c.cov.e02 << "  ";
				ply << c.cov.e11 << "  ";
				ply << c.cov.e12 << "  ";
				ply << c.cov.e22 << "  ";
				ply << c.weight << "  "; 
				ply << c.nvar.x << "  ";
				ply << c.nvar.y << "  ";
				ply << c.nvar.z << "  ";
				ply << endl;
			}

			plyString = ply.str();
		}
		

		/// load mixture from ascii PLY file format string.
		/// if non-empty mixture is passed, the loaded components are directly appended to the existing ones.
		static void importMixture(const string& plyString, Mixture& mixture)
		{
			typedef enum
			{
				X, Y, Z, COVXX, COVXY, COVXZ, COVYY, COVYZ, COVZZ, W, NVX, NVY, NVZ

			} MixtureElem;

			string line;
			int counter = 0;
			int componentCount = 0;
			bool readData = false;
			int lineCount = 0;
			vector<MixtureElem> header;

			// breacket first line
			string::size_type lineStart = 0;
			string::size_type lineEnd = plyString.find_first_of("\n", lineStart);

			while (lineStart != string::npos)
			{
				// extract line
				line = plyString.substr(lineStart, lineEnd - lineStart);

				// tokenize line
				string::size_type pos1 = line.find_first_not_of(" \n\r\t", 0);
				string::size_type pos2 = line.find_first_of(" \n\r\t", pos1);
				if (pos1 != string::npos && line.substr(pos1, pos2 - pos1).compare("comment") != 0)
				{
					// "element" command			
					if (line.substr(pos1, pos2 - pos1).compare("element") == 0)
					{
						pos1 = line.find_first_not_of(" \n\r\t", pos2);
						pos2 = line.find_first_of(" \n\r\t", pos1);

						if (line.substr(pos1, pos2 - pos1).compare("component") == 0)
						{
							pos1 = line.find_first_not_of(" \n\r\t", pos2);
							pos2 = line.find_first_of(" \n\r\t", pos1);
							componentCount = atoi(line.substr(pos1, pos2 - pos1).c_str());

							//cout << "Vertex Count: " << vertexCount << endl;
						}
					}
					// "property" command			
					else if (line.substr(pos1, pos2 - pos1).compare("property") == 0)
					{
						// read "float"
						pos1 = line.find_first_not_of(" \n\r\t", pos2);
						pos2 = line.find_first_of(" \n\r\t", pos1);
						
						// read property name
						pos1 = line.find_first_not_of(" \n\r\t", pos2);
						pos2 = line.find_first_of(" \n\r\t", pos1);
						string propName = line.substr(pos1, pos2 - pos1);
						
						MixtureElem elem;
						if (propName.compare("x") == 0) elem = X;
						else if (propName.compare("y") == 0) elem = Y;
						else if (propName.compare("z") == 0) elem = Z;
						else if (propName.compare("covxx") == 0) elem = COVXX;
						else if (propName.compare("covxy") == 0) elem = COVXY;
						else if (propName.compare("covxz") == 0) elem = COVXZ;
						else if (propName.compare("covyy") == 0) elem = COVYY;
						else if (propName.compare("covyz") == 0) elem = COVYZ;
						else if (propName.compare("covzz") == 0) elem = COVZZ;
						else if (propName.compare("weight") == 0) elem = W;
						else if (propName.compare("nvx") == 0) elem = NVX;
						else if (propName.compare("nvy") == 0) elem = NVY;
						else if (propName.compare("nvz") == 0) elem = NVZ;
						header.push_back(elem);
					}
					// element data
					else if (readData && counter++ < componentCount)
					{
						Component c;
						vec3 µ;
						smat3 cov;
						float w;
						vec3 nvar;

						for (int i = 0; pos1 != string::npos; ++i)
						{
							float val = (float)atof(line.substr(pos1, pos2 - pos1).c_str());

							switch (header[i])
							{
							case X: µ.x = val; break;
							case Y: µ.y = val; break;
							case Z: µ.z = val; break;
							case COVXX: cov.e00 = val; break;
							case COVXY: cov.e01 = val; break;
							case COVXZ: cov.e02 = val; break;
							case COVYY: cov.e11 = val; break;
							case COVYZ: cov.e12 = val; break;
							case COVZZ: cov.e22 = val; break;
							case W: w = val; break;
							case NVX: nvar.x = val; break;
							case NVY: nvar.y = val; break;
							case NVZ: nvar.z = val; break;
							}

							pos1 = line.find_first_not_of(" \n\r\t", pos2);
							pos2 = line.find_first_of(" \n\r\t", pos1);
						}
						mixture.push_back(Component(Gaussian(µ, cov, w), nvar));
					}
					// "end_header"
					else if (line.substr(pos1, pos2 - pos1).compare("end_header") == 0)
					{
						readData = true;
					}
				}

				// bracket next line
				lineStart = plyString.find_first_not_of("\n", lineEnd);
				lineEnd = plyString.find_first_of("\n", lineStart);
			}
		}



	};	/// end class PlyConverter


}	/// end namespace cp



