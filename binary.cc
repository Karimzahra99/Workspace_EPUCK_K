#include <iostream>
#include <string>
#include <bits/stdc++.h>
#include <limits>

using namespace std;
void showbits( unsigned int x ); //dec to bin
void decToHex(int n);
void DecToBin(int n);
void HexToBin(string hexdec);
void BinToHex(string bin);
void createMap(unordered_map<string, char> *um);
void BinToDec(int n); 
void HexToDec(string num);

template<typename T>
void showMinMax() {
   cout << "min: " << numeric_limits<T>::min() << endl;
   cout << "max: " << numeric_limits<T>::max() << endl;
   cout << endl;
}

int main( void )
{

    //showbits(~(3 << (1 * 2)));
    //DecToHex(10);
    //decToBin(40000);
    //HexToBin("100");
    //BinToHex("00011111");
    //BinToDec(1001);
    //HexToDec("3F456A");

   cout << "short:" << endl;
   showMinMax<short>();
   cout << "int:" << endl;
   showMinMax<int>();
   cout << "long:" << endl;
   showMinMax<long>();
   cout << "float:" << endl;
   showMinMax<float>();
   cout << "double:" << endl;
   showMinMax<double>();
   cout << "long double:" << endl;
   showMinMax<long double>();
   cout << "unsigned short:" << endl;
   showMinMax<unsigned short>();
   cout << "unsigned int:" << endl;
   showMinMax<unsigned int>();
   cout << "unsigned long:" << endl;
   showMinMax<unsigned long>();
    
    return 0; 
    
}

void showbits( unsigned int x )
{
    int i=0;
    for (i = (sizeof(int) * 8) - 1; i >= 0; i--)
    {
       putchar(x & (1u << i) ? '1' : '0');
    }
    printf("\n");
}

void DecToHex(int n)
{
    // char array to store hexadecimal number
    char hexaDeciNum[100];
 
    // counter for hexadecimal number array
    int i = 0;
    while (n != 0) {
        // temporary variable to store remainder
        int temp = 0;
 
        // storing remainder in temp variable.
        temp = n % 16;
 
        // check if temp < 10
        if (temp < 10) {
            hexaDeciNum[i] = temp + 48;
            i++;
        }
        else {
            hexaDeciNum[i] = temp + 55;
            i++;
        }
 
        n = n / 16;
    }
 
    // printing hexadecimal number array in reverse order
    for (int j = i - 1; j >= 0; j--)
        cout << hexaDeciNum[j];
}

void DecToBin(int n) 
{ 
    // array to store binary number 
    int binaryNum[32]; 
  
    // counter for binary array 
    int i = 0; 
    while (n > 0) { 
  
        // storing remainder in binary array 
        binaryNum[i] = n % 2; 
        n = n / 2; 
        i++; 
    } 
  
    // printing binary array in reverse order 
    for (int j = i - 1; j >= 0; j--) 
        cout << binaryNum[j]; 
} 

void HexToBin(string hexdec)
{
    long int i = 0;
 
    while (hexdec[i]) {
 
        switch (hexdec[i]) {
        case '0':
            cout << "0000";
            break;
        case '1':
            cout << "0001";
            break;
        case '2':
            cout << "0010";
            break;
        case '3':
            cout << "0011";
            break;
        case '4':
            cout << "0100";
            break;
        case '5':
            cout << "0101";
            break;
        case '6':
            cout << "0110";
            break;
        case '7':
            cout << "0111";
            break;
        case '8':
            cout << "1000";
            break;
        case '9':
            cout << "1001";
            break;
        case 'A':
        case 'a':
            cout << "1010";
            break;
        case 'B':
        case 'b':
            cout << "1011";
            break;
        case 'C':
        case 'c':
            cout << "1100";
            break;
        case 'D':
        case 'd':
            cout << "1101";
            break;
        case 'E':
        case 'e':
            cout << "1110";
            break;
        case 'F':
        case 'f':
            cout << "1111";
            break;
        default:
            cout << "\nInvalid hexadecimal digit "
                 << hexdec[i];
        }
        i++;
    }
}

void createMap(unordered_map<string, char> *um)
{
    (*um)["0000"] = '0';
    (*um)["0001"] = '1';
    (*um)["0010"] = '2';
    (*um)["0011"] = '3';
    (*um)["0100"] = '4';
    (*um)["0101"] = '5';
    (*um)["0110"] = '6';
    (*um)["0111"] = '7';
    (*um)["1000"] = '8';
    (*um)["1001"] = '9';
    (*um)["1010"] = 'A';
    (*um)["1011"] = 'B';
    (*um)["1100"] = 'C';
    (*um)["1101"] = 'D';
    (*um)["1110"] = 'E';
    (*um)["1111"] = 'F';
}
 
// function to find hexadecimal 
// equivalent of binary
void BinToHex(string bin)
{
    int l = bin.size();
    int t = bin.find_first_of('.');
     
    // length of string before '.'
    int len_left = t != -1 ? t : l;
     
    // add min 0's in the beginning to make
    // left substring length divisible by 4 
    for (int i = 1; i <= (4 - len_left % 4) % 4; i++)
        bin = '0' + bin;
     
    // if decimal point exists    
    if (t != -1)    
    {
        // length of string after '.'
        int len_right = l - len_left - 1;
         
        // add min 0's in the end to make right
        // substring length divisible by 4 
        for (int i = 1; i <= (4 - len_right % 4) % 4; i++)
            bin = bin + '0';
    }
     
    // create map between binary and its
    // equivalent hex code
    unordered_map<string, char> bin_hex_map;
    createMap(&bin_hex_map);
     
    int i = 0;
    string hex = "";
     
    while (1)
    {
        // one by one extract from left, substring
        // of size 4 and add its hex code
        hex += bin_hex_map[bin.substr(i, 4)];
        i += 4;
        if (i == bin.size())
            break;
             
        // if '.' is encountered add it
        // to result
        if (bin.at(i) == '.')    
        {
            hex += '.';
            i++;
        }
    }
    cout << hex << endl;
    // required hexadecimal number
    
}

void BinToDec(int n) //int num = 10101001;
{
    int num = n;
    int dec_value = 0;
 
    // Initializing base value to 1, i.e 2^0
    int base = 1;
 
    int temp = num;
    while (temp) {
        int last_digit = temp % 10;
        temp = temp / 10;
 
        dec_value += last_digit * base;
 
        base = base * 2;
    }
 
    //return dec_value;
    cout << dec_value << endl;
}

void HexToDec(string num) { //char num[] = "3F456A";
   //int len = strlen(num);
   int len = num.length();
   int base = 1;
   int temp = 0;
   for (int i=len-1; i>=0; i--) {
      if (num[i]>='0' && num[i]<='9') {
         temp += (num[i] - 48)*base;
         base = base * 16;
      }
      else if (num[i]>='A' && num[i]<='F') {
         temp += (num[i] - 55)*base;
         base = base*16;
      }
   }
   //return temp;
   cout << temp << endl;
}
