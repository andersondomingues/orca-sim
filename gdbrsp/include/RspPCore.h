#ifndef _RSP_PCORE_H
#define _RSP_PCORE_H

/**
 * Abstract class that models the interface for a 
 * RSP server-compliant processor core. Processor
 * cores that are intend to communicate with RSP
 * clients must inherit from this class and implement
 * the whole set of virtual methods. */
class RspPCore{

public:

    RspPCore();

    virtual RspResponse* Handle_C(RspRequest*);
    virtual RspResponse* Handle_G(RspRequest*);

    virtual ~RspPCore();

};




#endif /* _RSP_PCORE_H */
