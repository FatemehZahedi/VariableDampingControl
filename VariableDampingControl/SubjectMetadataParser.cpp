#include <SubjectMetadataParser.h>

bool ParseSubjectMetadataFile(SubjectMetadata & smds, std::string filepath){
    /* Determine if file exists */
    if (FILE * file = fopen(filepath.c_str(), "r")){
        fclose(file);
    }
    else{
        printf("Subject Meta File Not Found\nFile: %s \n", filepath.c_str());
        printf("\nRESULTS WILL BE RECORDED IN DEFAULT H5 FILE\n");
        return false;
    }

    /* Read in and parse file */
    std::ifstream file(filepath.c_str());
    bool hasName = false;
    bool hasH5Filepath = false;
    bool hasSubjectNumber = false;
	std::string line;
    while (getline(file, line)){
        /*
        Each line is assumed to have the form key: value
        To process:
        1. Split at ":" delimiter
        2. Compare the key value
        3. Strip whitespace off value string
        4. Store value in SubjectMetadata struct
        */
        size_t ind = line.find(":");

        if (ind == std::string::npos){ /* Line does not have ":" delimiter */
            printf("Line does not have correct form\n");
            printf("Inputted line: %s\n", line.c_str());
            printf("Correct form:\nkey: value\n");
        }
        else{                          /* Line has ":" delimiter */
            /* Separate key and value from line */
            std::string key = line.substr(0, ind);
            std::string valstr = line.substr((ind+1), valstr.length() - (ind+1));
            /* Strip whitespace off front of valstr*/
            size_t first_char_pos = valstr.find_first_not_of(" ");
            valstr = valstr.substr(first_char_pos, valstr.length() - first_char_pos);

            /* Make key string uppercase so comparing is easier*/
            std::transform(key.begin(), key.end(), key.begin(), ::toupper);

            /* Compare key string and assign values to SubjectMetdata struct */
            if (key.compare("NAME") == 0){
                smds.Name = valstr;
                hasName = true;
            }
            else if (key.compare("NUMBER") == 0){
            	smds.Number = atoi(valstr.c_str());
                hasSubjectNumber = true;
            }
            else if (key.compare("AGE") == 0){
            	smds.Age = atoi(valstr.c_str());
            }
            else if (key.compare("HEIGHT") == 0){
            	smds.Height = atoi(valstr.c_str());
			}
            else if (key.compare("WEIGHT") == 0){
            	smds.Weight = atof(valstr.c_str());
            }
            else if (key.compare("GENDER") == 0){
            	smds.Gender = valstr;
            }
            else if (key.compare("H5FILE") == 0){
            	smds.H5File = valstr;
                hasH5Filepath = true;
            }
            else if (key.compare("USEEMG") == 0){
                std::transform(valstr.begin(), valstr.end(), valstr.begin(), ::toupper);
                if (valstr.compare("TRUE")){
                    smds.UseEmg = true;
                }
            }
            else{
                printf("Key does not match anything --- SKIPPING\n");
                printf("Line: %s\n", line.c_str());
                printf("Key: %s\n\n", key.c_str());
            }
        }
    }
    /* Return whether or not the file has enough info for H5 logging*/
    if (hasSubjectNumber && hasName && hasH5Filepath){
        return true;
    }
    else{
        printf("hasName: %s\n", smds.Name.c_str());
        printf("hasH5Filepath: %s\n", smds.H5File.c_str());
        printf("hasSubjectNumber: %d\n", smds.Number);
        printf("File found, but not enough information specified in file\n");
        printf("RESULTS WILL BE RECORDED IN DEFAULT H5 FILE\n");
        return false;
    }
}
