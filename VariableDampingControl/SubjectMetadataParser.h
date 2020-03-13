#include <fstream>
#include <algorithm>
#include <string>
#include <stdio.h>


typedef struct SubjectMetadata{
    std::string Name;
    int Number;
    int Age = 0;
    int Height = 0;
    double Weight = 0;
    std::string Gender = "";
    std::string H5File;
    bool UseEmg = false;
};


bool ParseSubjectMetadataFile(SubjectMetadata & smds, std::string filepath);
