#ifndef _ORCA_SIM_H
#define _ORCA_SIM_H

class OrcaSim{

private:
	Tile _tiles[ORCA_NOC_WIDTH][ORCA_NOC_HEIGHT];

public:
	void Setup();
	void Execute();
	void Report();
};


#endif
