/* UMANS: Unified Microscopic Agent Navigation Simulator
** Copyright (C) 2018-2020  Inria Rennes Bretagne Atlantique - Rainbow - Julien Pettré
**
** This program is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program. If not, see <https://www.gnu.org/licenses/>.
**
** Contact: crowd_group@inria.fr
** Website: https://project.inria.fr/crowdscience/
** See the file AUTHORS.md for a list of all contributors.
*/

//========================================================================
/*!
  @file		csvwriter.cpp
  @class	CSVWriter
  @date    	24/8/2018
  @brief
  @author  Javad Amirian, (C) 2018
*/
//========================================================================

#include <tools/csvwriter.h>
#include <tools/HelperFunctions.h>

#include <fstream>

bool CSVWriter::SetOutputDirectory(const std::string &dirname)
{
	// if necessary, append a slash to the directory path
	if (dirname.back() == '/')
		this->dirname_ = dirname;
	else
		this->dirname_ = dirname + '/';

	// try to create the directory, and return if the directory exists
	HelperFunctions::CreateDirectoryIfNonExistent(this->dirname_);
	return HelperFunctions::DirectoryExists(this->dirname_);
}

bool CSVWriter::Flush()
{
	if (dirname_.empty())
		return false;

    mtx_.lock();
    std::map<int, std::vector<TimeAndPos>> pos_log_copy = pos_log_;
    pos_log_.clear();
    mtx_.unlock();

    for (auto itr=pos_log_copy.begin(); itr != pos_log_copy.end(); itr++) {
        std::fstream file_i;
        int id = itr->first;
        std::string file_name = dirname_ + "output_" + std::to_string(id) + ".csv";
        file_i.open(file_name, std::ios::app);
        for (size_t i=0; i<itr->second.size(); i++)
            file_i << itr->second[i].t_ << "," << itr->second[i].x_ << "," << itr->second[i].y_ << "\n";
        file_i.close();
        itr->second.clear();
    }

	return true;
}
void CSVWriter::AppendAgentPositions(std::map<int, Vector2D> &poss, double t)
{
    mtx_.lock();
    for(auto itr=poss.begin(); itr != poss.end(); itr++)
        pos_log_[itr->first].push_back(TimeAndPos(t, itr->second.x, itr->second.y));
    mtx_.unlock();

	if (flushImmediately)
		Flush();
}

CSVWriter::~CSVWriter()
{
	Flush();
}