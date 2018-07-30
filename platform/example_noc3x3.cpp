#include <Event.h>
#include <Simulator.h>

#include <NocRouterModel.h>

#define CYCLES_TO_SIM 100000

/*mesh size*/
#define X 3
#define Y 3 

//prototypes
void printBuffers();

//local buffers for each router
Buffer* buffers[X][Y];    

//instantiates a mesh of MxM routers
NocRouterModel* routers[X][Y];

//this program tests the NocRouterModel class 
int main(int argc, char** argv){

	cout << "Simulation stated" << endl;
	
	//creates a new simulator
	Simulator s = Simulator(CYCLES_TO_SIM);

    for(int i = 0; i < X; i++){
        for(int j = 0; j < Y; j++){
            std::string bname = "(" + std::to_string(i) + "," + std::to_string(j) + ").IN" + std::to_string(LOCAL);
            buffers[i][j] = new Buffer(bname);
        }
    }
    
    for(int i = 0; i < X; i++){
        for(int j = 0; j < Y; j++){
            routers[i][j] = new NocRouterModel("router1", i, j); //instantiate new router
            routers[i][j]->SetInputBuffer(
                buffers[i][j], 
                LOCAL
            ); //attach a buffer to the input
        }
    }
    
    /*------------------------
     *   (0,2)  (1,2)  (2,2)
     *   
     *   (0,1)  (1,1)  (2,1)
     * 
     *   (0,0)  (1,0)  (2,0)
     *------------------------
    //bind routers to the left */
    for(int i = 1; i < X; i++)
        for(int j = 0; j < Y; j++)
            routers[i][j]->SetInputBuffer(
                routers[i -1][j]->GetOutputBuffer(EAST),
                WEST
            );

    //bind routers to the right
    for(int i = 0; i < X-1; i++)
        for(int j = 0; j < Y; j++)
            routers[i][j]->SetInputBuffer(
                routers[i +1][j]->GetOutputBuffer(WEST),
                EAST
            );

    //bind routers to the top
    for(int i = 0; i < X; i++)
        for(int j = 1; j < Y; j++)
            routers[i][j]->SetInputBuffer(
                routers[i][j-1]->GetOutputBuffer(NORTH), 
                SOUTH
            );
    
    //bind routers to the bottom
    for(int i = 0; i < X; i++)
        for(int j = 0; j < Y -1; j++)
            routers[i][j]->SetInputBuffer(
                routers[i][j+1]->GetOutputBuffer(SOUTH), 
                NORTH
            );
    
    //schedule router to be simulated
    for(int i = 0; i < X; i++)
        for(int j = 0; j < Y; j++)
            s.Schedule(Event(0, routers[i][j]));
            

    //--------- begin of testcase ------------------------
    //add a package (0,0, 0x0) to router (2,2)
    buffers[2][2]->push(0x0);

    //print buffers
    printBuffers();

    s.Run();
    
    //print buffers
    printBuffers();
    //----------------------------------------------------
    
	cout << "\nSimulation ended" << endl;
}

//print all buffers
void printBuffers(){
   cout << "router\toutput\tinput" << endl;
    for(int i = 0; i < X; i++){
        for(int j = 0; j < Y; j++) {
            std::cout << "(" << i << "," << j << ")\t";
            std::cout << routers[i][j]->GetOutputBuffer(LOCAL)->size() << "\t";
            std::cout << routers[i][j]->GetInputBuffer(LOCAL)->size() << std::endl;
        }
   }
}
