

// Samples


// c++
#include <vector>
#include <iostream>


// 



using namespace std;




class class_name
{
private:

public:
    


    int onInitialize() 
    {

        return -1;
    }

    
    int onReset() 
    {
        return -1;
    }
    

    int onRelease() 
    {
       return -1;
    }
    int onSubscribe()
    {
    }
    int onProcess() 
    {
        
        return -1;
    }
    int onRender() 
    {  
       
        return -1;
  
    }
};


int main()
{
    class_name class_obj;
    class_obj.onInitialize();
    
    while(true)
    {
        
        
        class_obj.onProcess();
        class_obj.onRender();
        
    }
    class_obj.onReset();
    class_obj.onRelease();
    return 0;

}
