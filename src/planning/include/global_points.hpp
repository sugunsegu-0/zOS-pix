#include <iostream>
#include <map>
#include <fstream>
#include <string>
std::map<std::string, std::vector<std::vector<float>>> waypoints;

void read_points()
{
    std::fstream newfile;
   int i=0;
   newfile.open("tpoint.txt",ios::in); //open a file to perform read operation using file object
   if (newfile.is_open()){ //checking whether the file is open
      string tp;
      std::vector<std::vector<float>> out;
      std::vector<float> out_in;
      while(getline(newfile, tp)){ //read data from file object and put it into string.

        if(tp=="a")
        {
            
            waypoints.insert(pair<std::string, std::vector<std::vector<float>>>(to_string(i), out));
            out.clear();
        }
        else if(i%2==0)
        {
            out_in.push_back(std::stod(tp));
            cout <<"x" <<tp << "\n"; //print the data of the string
        }
        else if(i%2!=0)
        {
            out_in.push_back(std::stod(tp));
            out.push_back(out_in);
            out_in.clear();
            cout << "y" <<tp << "\n"; //print the data of the string
        }


      }
   }
    newfile.close(); //close the file object.
    
}
