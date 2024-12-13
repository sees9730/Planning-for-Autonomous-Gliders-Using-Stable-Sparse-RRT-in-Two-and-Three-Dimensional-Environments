#include <iostream>

#include <string>

#include <fstream>

#include <cmath>

#include <iomanip>

using namespace std;


//global variables 

string name;

string month;

int year;



//Function 1: File input 

void func1(int N, string filename, int birthyear)

{

    

    ifstream inputFile;

    inputFile.open(filename);

    if (inputFile.fail())

    {

        cout <<"Error with opening file.";

    }

    for(int i=0;i<N;i++)

    {

        inputFile >>name >>month >>year;

        //Identify who's birthyear is later 

        if (year > birthyear)

        {

            cout <<name <<endl;

        }

    }

}


//Function 2: Reorder values

void func2(double x, double y, double z)

{

    double temp;

    //If statements to switch 

    if(x>y)

    {

        temp = x;

        x = y;

        y = temp;

    }

    if (y>z)

    {

        temp = z;

        z = y;

        y = temp;

    }

    //Give results 

    cout <<x <<y <<z <<endl;

}


//Function 3: Calculate Volume of sphere

// void func3(int N, double radius[], int volume[])

// {

//     for(int i=0; i<N; i++)

//     {

//         volume[N] = (4/3) * M_PI * (radius[N]*radius[N]*radius[N]);

      

//         if (volume[N] % 2 != 0)

//         {

//             volume[N]++;

            

//         }

//     }

// }


int main() {

    //Local variables 

    string filename;

    int birthyear;

    int N = 4;

    double x;

    double y;

    double z;

    double radius[N];

    int volume[N];

    

    //Output 

    cout <<"Enter (1) the file name (2) the number of rows in the file and (3) the year of birth: ";

    cin >>filename >>N >>birthyear;

    func1(N,filename,birthyear);

    cout <<endl;

    //function 2

    cout <<"Enter X, Y, and Z: ";

    cin >>x >>y >>z;

    func2(x,y,z);

    cout <<"Enter (1) the array size and (2) the radius: ";

    cin >> N >>radius[N];

    // func3 (N, radius[N], volume[N]);

    // for(int i=0;i<N;i++)

    // {
// 
        // cout <<volume[N] <<endl;

    // }

    

    

  

    

  

  

 

    

    

    

    return 0;

}
