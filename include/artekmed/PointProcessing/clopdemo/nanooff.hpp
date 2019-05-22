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


#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;



struct nanooff
{
	class Info
	{
		bool signatureOK;
		uint vertexCount;

	public:
		bool getSignatureOK() const { return signatureOK; }
		uint getVertexCount() const { return vertexCount; }

	public:
		Info (const string& filename)
		{
			// read file
			std::ifstream in(filename);
			std::stringstream buffer;
			buffer << in.rdbuf();
			in.close();
			string ascii = buffer.str();


			// 1: bracket first line
			string::size_type lineStart = 0;
			string::size_type lineEnd = ascii.find_first_of("\n", lineStart);
			signatureOK = ascii.substr(lineStart, lineEnd).compare("OFF") == 0;
			
			// bracket next line: vertices, faces edges
			lineStart = ascii.find_first_not_of("\n", lineEnd);
			lineEnd = ascii.find_first_of("\n", lineStart);
			string line = ascii.substr(lineStart, lineEnd - lineStart);
			string::size_type pos1 = line.find_first_not_of(" \n\r\t", 0);
			string::size_type pos2 = line.find_first_of(" \n\r\t", pos1);
			vertexCount = (pos2 == pos1) ? 0 : atoi(line.substr(pos1, pos2 - pos1).c_str());
		}
	};



	static bool loadPointCloud(const string& filename, float* buffer)
	{
		string line;
		string::size_type lineStart, lineEnd, pos1, pos2;
		uint vertexCount = 0;
		
		// read file
		std::ifstream in(filename);
		std::stringstream strbuffer;
		strbuffer << in.rdbuf();
		in.close();
		string ascii = strbuffer.str();

		// 1: bracket first line
		lineStart = 0;
		lineEnd = ascii.find_first_of("\n", lineStart);
		if (ascii.substr(lineStart, lineEnd).compare("OFF"))
			return false;

		// bracket next line: vertices, faces edges
		lineStart = ascii.find_first_not_of("\n", lineEnd);
		lineEnd = ascii.find_first_of("\n", lineStart);
		line = ascii.substr(lineStart, lineEnd - lineStart);
		pos1 = line.find_first_not_of(" \n\r\t", 0);
		pos2 = line.find_first_of(" \n\r\t", pos1);
		vertexCount = atoi(line.substr(pos1, pos2 - pos1).c_str());
		if (vertexCount == 0)
			return false;


		// read vertices
		lineStart = ascii.find_first_not_of("\n", lineEnd);
		lineEnd = ascii.find_first_of("\n", lineStart);

		uint lineNr;
		for (lineNr = 0; lineNr < vertexCount && lineStart != string::npos; ++lineNr)
		{
			// extract line
			line = ascii.substr(lineStart, lineEnd - lineStart);

			// get X
			pos1 = line.find_first_not_of(" \n\r\t", 0);
			pos2 = line.find_first_of(" \n\r\t", pos1);
			*buffer++ = (float)atof(line.substr(pos1, pos2 - pos1).c_str());

			// get Y
			pos1 = line.find_first_not_of(" \n\r\t", pos2);
			pos2 = line.find_first_of(" \n\r\t", pos1);
			*buffer++ = (float)atof(line.substr(pos1, pos2 - pos1).c_str());

			// get Z
			pos1 = line.find_first_not_of(" \n\r\t", pos2);
			pos2 = line.find_first_of(" \n\r\t", pos1);
			*buffer++ = (float)atof(line.substr(pos1, pos2 - pos1).c_str());


			// bracket next line
			lineStart = ascii.find_first_not_of("\n", lineEnd);
			lineEnd = ascii.find_first_of("\n", lineStart);
		}


		// see if we could really read everything
		if (lineNr < vertexCount)
			return false;

		return true;
	}



	static bool savePointCloud(const string& filename, float* buffer, uint nPoints)
	{
		stringstream ascii;

		ascii << "OFF\n";
		ascii << nPoints << " 0 0\n";

		float *ptr = buffer;
		for (uint i = 0; i < nPoints; ++i)
			ascii << buffer[3*i] << " " << buffer[3*i + 1] << " " << buffer[3*i + 2] << endl;

		// write file
		std::ofstream out(filename);
		out << ascii.str() << endl;
		out.close();

		return true;
	}
};
