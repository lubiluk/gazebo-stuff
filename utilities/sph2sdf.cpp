//
// Created by lubiluk on 5/25/17.
//
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cmath>
#include <limits>
#include <algorithm>

using namespace std;

class tree_layout {
public:
    unsigned levels_number = 0;
    unsigned branching_factor = 0;

    tree_layout(unsigned levels_number, unsigned branching_factor):
            levels_number(levels_number),
            branching_factor(branching_factor) {}

    tree_layout() {};

    /// Returns the number of line of the first leaf
    unsigned get_leaf_start_line() {
        // Add one to the get_start_line to start from the next line
        unsigned start_line = 1;

        for (int i = 0; i< (levels_number-1); ++i) {
            start_line += pow(branching_factor, i);
        }

        return start_line;
    }
};

int main(int argc, char** argv)
{
    if (argc != 4) {
        cout << "Usage: sph2sdf SOURCE DEST VISUAL\n";
        return 0;
    }

    char* source = argv[1];
    char* dest = argv[2];
    string visual(argv[3]);

    std::transform(visual.begin(), visual.end(), visual.begin(), ::tolower);

    bool generate_visual = (visual == "1" || visual == "true");

    ifstream source_file(source);
    ofstream dest_file(dest);

    string line;
    unsigned index = 0;

    tree_layout series;

    if (!(source_file >> series.levels_number >> series.branching_factor)) {
        cerr << "Cannot read input file\n";
        return -1;
    };

    // Skip first irrelevant lines
    auto line_number = series.get_leaf_start_line();

    for (unsigned i = 0; i< line_number; ++i) {
        source_file.ignore(numeric_limits<streamsize>::max(), '\n');
    }

    // Convert remaining entries to xml
    dest_file << "<?xml version='1.0'?>\n";
    dest_file << "<sdf version='1.6'>\n";
    dest_file << "\t<model name='" << source << "'>\n";
    dest_file << "\t<static>false</static>\n";
    dest_file << "\t<pose>0 0 0 0 0 0</pose>\n";
    dest_file << "\t\t<link name='link'>\n";

    while (getline(source_file, line)) {
        istringstream line_stream(line);

        double x, y, z, r, s;
        x = y = z = r = s = 0.0;

        if (!(line_stream >> x >> y >> z >> r >> s)) continue;
        if (x == 0.0 && y == 0.0 && z == 0.0) continue;

        dest_file << "\t\t\t<collision name='collision_" << index << "'>\n"
                     "\t\t\t\t<pose>" << x << " " << y << " " << z << " 0 0 0</pose>\n"
                     "\t\t\t\t<geometry>\n"
                     "\t\t\t\t\t<sphere>\n"
                     "\t\t\t\t\t\t<radius>" << r << "</radius>\n"
                     "\t\t\t\t\t</sphere>\n"
                     "\t\t\t\t</geometry>\n"
                     "\t\t\t</collision>\n";

        if (generate_visual) {
            dest_file << "\t\t\t<visual name='visual_" << index << "'>\n"
                         "\t\t\t\t<pose>" << x << " " << y << " " << z << " 0 0 0</pose>\n"
                         "\t\t\t\t<geometry>\n"
                         "\t\t\t\t\t<sphere>\n"
                         "\t\t\t\t\t\t<radius>" << r << "</radius>\n"
                         "\t\t\t\t\t</sphere>\n"
                         "\t\t\t\t</geometry>\n"
                         "\t\t\t</visual>\n";
        }

        index += 1;
    }

    dest_file << "\t\t</link>\n";
    dest_file << "\t</model>\n";
    dest_file << "</sdf>\n";

    return 0;
}