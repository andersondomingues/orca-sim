#ifndef _packet
#define _packet

//------------------------------------  NOC Definitions ----------------------------------

  #define EAST 0
  #define WEST 1
  #define NORTH 2
  #define SOUTH 3
  #define LOCAL 4

  //#define TAM_LINHA 2

  #define TAM_FLIT 16
  #define METADEFLIT (TAM_FLIT/2)
  #define QUARTOFLIT (TAM_FLIT/4)

  //#define NROT 9
  #define NPORT 5
  #define BUFFER_TAM 16

  typedef sc_uint<TAM_FLIT > regflit;
  typedef sc_uint<8 > regaddress;

  typedef sc_uint<3> reg3;
  typedef sc_uint<4> reg4;
  typedef sc_uint<8> reg8;
  typedef sc_uint<10> reg10;
  typedef sc_uint<11> reg11;
  typedef sc_uint<16> reg16;
  typedef sc_uint<32> reg32;
  typedef sc_uint<40> reg40;
  //typedef sc_uint<NROT> regNrot;
  typedef sc_uint<NPORT> regNport;
  typedef sc_uint<TAM_FLIT> regflit;
  typedef sc_uint<(TAM_FLIT/2)> regmetadeflit;
  typedef sc_uint<(TAM_FLIT/4)> regquartoflit;
  typedef sc_uint<(3*NPORT)> reg_mux;
//  typedef sc_uint<(NPORT-1)><regflit> arrayNport_regflit;

#endif
