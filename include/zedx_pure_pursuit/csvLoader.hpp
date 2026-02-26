#pragma once

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class wayPoint 
{
public:
    double x, y; // Add more attributes as needed

    wayPoint(double x, double y)
    {
        this->x = x;
        this->y = y;
    }
};

class wayPointLoader
{
public:
    std::vector<wayPoint> way_points;
    
    wayPointLoader(const std::string file_path)
    {
        std::ifstream file(file_path);

        if(!file.is_open()) throw std::runtime_error("could not open csv file" + file_path);

        std::string line;
        while (getline(file, line)) 
        {
            this->way_points.push_back(parse_csv_line(line));
        }
    }

private:
    wayPoint parse_csv_line(const std::string line)
    {
        std::istringstream s(line);
        std::string field;

        double x, y;

        // Assuming CSV format is: x,y
        getline(s, field, ',');
        x = std::stod(field);
        
        getline(s, field);
        y = std::stod(field);

        return wayPoint(x, y);
    }
};