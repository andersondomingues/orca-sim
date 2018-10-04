/***
*	Plasma
***/
#ifndef _plasma_master_h
#define _plasma_master_h

#include "systemc.h"

#include "../../plasma/sc/mlite_cpu.h"
#include "../../router/sc/packet.h"
#include "../../router/sc/router_cc.h"

#include <ram_master.h>

#include "../../dma/sc/dma_master.h"

#include "../../ni/sc/Network_Interface.h"
#include "../../plasma/sc/access_repository.h"

// Memory map constants.
#define DEBUG 0x20000000
#define IRQ_MASK 0x20000010
#define IRQ_STATUS_ADDR 0x20000020
#define COUNTER 0x20000060
#define FIFO_AVAIL 0x20000040
#define END_SIM 0x20000080

// Netwok interface mapping.
#define NI_STATUS_READ 0x20000100
#define NI_STATUS_SEND 0x20000110
#define NI_READ 0x20000120
#define NI_WRITE 0x20000130
#define NI_CONFIGURATION 0x20000140
#define NI_ACK 0x20000150
#define NI_NACK 0x20000160
#define NI_END 0x20000170
// DMA mapping.
#define DMA_SIZE 0x20000200
#define DMA_ADDR 0x20000210
#define DMA_OP 0x20000220
#define START_DMA 0x20000230
#define DMA_ACK 0x20000240
#define DMA_ACTIVE 0x20000250
#define TICK_COUNTER_ADDR 0x20000300
// Dynamic Insertion of Applications
#define REQ_TASK_REG 0x20000350
#define ACK_TASK_REG 0x20000360
#define SYSCALL_ADDR 0x20000500


SC_MODULE(plasma_master) {
	
	sc_in< bool >	clock;
	sc_in< bool >	reset;
		
	// NoC Interface
	sc_out<bool >		clock_tx[NPORT-1];
	sc_out<bool >		tx[NPORT-1];
	sc_out<regflit >	data_out[NPORT-1];
	sc_in<bool >		credit_i[NPORT-1];
	
	sc_in<bool >		clock_rx[NPORT-1];
	sc_in<bool > 		rx[NPORT-1];
	sc_in<regflit >		data_in[NPORT-1];
	sc_out<bool >		credit_o[NPORT-1];
	
	// External Memory
	sc_out< sc_uint<30> >	address;
	sc_out< bool >			read_req;
	sc_out< sc_uint<32> >	data_write;
	sc_in< sc_uint<32> > 	data_read;
	sc_out< sc_uint<4> >	write_byte_enable;
	sc_in< bool > 			data_valid;
		
	// Debug MC
	sc_out< bool >			write_enable_debug;
	sc_out< sc_uint<32> >	data_out_debug;
	sc_in< bool >			busy_debug;
	
	//Dynamic Insertion of Applications
	sc_out<bool >			ack_task;
	sc_in<sc_uint<32> >		req_task;
	
	//signals
	sc_signal < sc_uint <32 > > cpu_mem_address_reg;
	sc_signal < sc_uint <32 > > cpu_mem_data_write_reg;
	sc_signal < sc_uint <4 > > cpu_mem_write_byte_enable_reg;
	sc_signal < sc_uint <8 > > irq_mask_reg;
	sc_signal < sc_uint <8 > > irq_status;
	sc_signal < bool > irq;
	sc_signal < sc_uint <32 > > time_slice;
	sc_signal < bool > write_enable;
	sc_signal < bool > syscall;
	sc_signal < sc_uint <32 > > tick_counter;
	sc_signal < sc_uint <8 > > current_page;
	//cpu
	sc_signal < sc_uint <32 > > cpu_mem_address;
	sc_signal < sc_uint <32 > > cpu_mem_data_write;
	sc_signal < sc_uint <32 > > cpu_mem_data_read;
	sc_signal < sc_uint <4 > > cpu_mem_write_byte_enable;
	sc_signal < bool > cpu_mem_pause;
	sc_signal < bool > cpu_enable_external_ram;
	sc_signal < bool > cpu_enable_internal_ram;
	sc_signal < bool > cpu_read_data;
	sc_signal < bool > cpu_send_data;
	sc_signal < bool > cpu_packet_ack;
	sc_signal < bool > cpu_packet_nack;
	sc_signal < bool > cpu_packet_end;
	sc_signal < bool > cpu_set_size;
	sc_signal < bool > cpu_set_address;
	sc_signal < bool > cpu_set_op;
	sc_signal < bool > cpu_start;
	sc_signal < bool > cpu_ack;
	//ram
	sc_signal < sc_uint <32 > > data_read_ram;
	sc_signal < sc_uint <32 > > mem_data_read;
	//mc debug 
	sc_signal < bool > debug_busy;
	sc_signal < bool > debug_write_data;
	sc_signal < bool > debug_write_busy;
	sc_signal < bool > debug_data_avail;
	sc_signal < sc_uint <8 > > data_read_debug;
	//network interface
	sc_signal < bool > plasma_hold;
	sc_signal < bool > ni_send_av;
	sc_signal < bool > ni_read_av;
	sc_signal < bool > ni_intr;
	sc_signal < bool > ni_send_data;
	sc_signal < bool > ni_read_data;
	sc_signal < sc_uint <32 > > ni_data_write;
	sc_signal < sc_uint <32 > > ni_data_read;
	sc_signal < sc_uint <32 > > ni_config;
	// NoC Interface
	sc_signal< bool > clock_tx_ni;
	sc_signal< bool > tx_ni;
	sc_signal< regflit > data_out_ni;
	sc_signal< bool > credit_i_ni;
	sc_signal< bool > clock_rx_ni;
	sc_signal< bool > rx_ni;
	sc_signal< regflit > data_in_ni;
	sc_signal< bool > credit_o_ni;
	//dma    
	sc_signal < bool > dma_read_data;
	sc_signal < bool > dma_send_data;
	sc_signal < bool > dma_intr;
	sc_signal < sc_uint <32 > > dma_mem_address;
	sc_signal < sc_uint <32 > > dma_mem_addr_ddr;
	sc_signal < bool > dma_mem_ddr_read_req;
	sc_signal < bool > mem_ddr_access;
	sc_signal < sc_uint <4 > > dma_mem_write_byte_enable;
	sc_signal < sc_uint <32 > > dma_mem_data_write;
	sc_signal < sc_uint <32 > > dma_mem_data_read;
	sc_signal < sc_uint <32 > > dma_data_read;
	sc_signal < sc_uint <32 > > dma_data_write;
	sc_signal < bool > dma_enable_internal_ram;
	sc_signal < bool > dma_active_sig;
	sc_signal < bool > data_avail_sig;
	sc_signal < bool > busy_sig;
	sc_signal < bool > mem_hold;
	sc_signal < bool > dma_waiting;
	sc_signal < sc_uint <32 > > data_read_reg;
	sc_signal < bool > empty_out;
	sc_signal < sc_uint <30 > > address_mux;
	sc_signal < sc_uint <30 > > addr_a;
	sc_signal < sc_uint <30 > > addr_b;
	
	sc_signal < sc_uint <32 > > end_sim_reg;

	//router signals
	//not reset for router 
	sc_signal < bool > reset_n;
	
	mlite_cpu *cpu;
	ram_master *ram_mas;
	dma_master *dma_mas;
	Network_Interface* ni;
	access_repository *repo_access;
	router_cc *router;
	
	void sequential_attr();
	void comb_assignments();
	void mem_mapped_registers();
	void reset_n_attr();
	void end_of_simulation();
	
	SC_HAS_PROCESS(plasma_master);
	plasma_master(sc_module_name name_, regaddress address_ = 0x00) :
	sc_module(name_), router_address(address_)
	{

		end_sim_reg.write(0x00000001);

		cpu = new mlite_cpu("mlite");
		cpu->clk(clock);
		cpu->reset_in(reset);
		cpu->intr_in(irq);
		cpu->mem_address(cpu_mem_address);
		cpu->mem_data_w(cpu_mem_data_write);
		cpu->mem_data_r(cpu_mem_data_read);
		cpu->mem_byte_we(cpu_mem_write_byte_enable);
		cpu->mem_pause(cpu_mem_pause);
		cpu->current_page(current_page);
		
		ram_mas = new ram_master("ram_master");
		ram_mas->clk(clock);
		ram_mas->enable_a(cpu_enable_internal_ram);
		ram_mas->wbe_a(cpu_mem_write_byte_enable);
		ram_mas->address_a(addr_a);
		ram_mas->data_write_a(cpu_mem_data_write);
		ram_mas->data_read_a(data_read_ram);
		ram_mas->enable_b(dma_enable_internal_ram);
		ram_mas->wbe_b(dma_mem_write_byte_enable);
		ram_mas->address_b(addr_b);
		ram_mas->data_write_b(dma_mem_data_write);
		ram_mas->data_read_b(mem_data_read);
		
		ni = new Network_Interface("ni", router_address);
		ni->clock(clock );
		ni->reset(reset);
		ni->clock_tx(clock_tx_ni);
		ni->tx(tx_ni);
		ni->data_out(data_out_ni);
		ni->credit_i(credit_i_ni);
		ni->clock_rx(clock_rx_ni);
		ni->rx(rx_ni);
		ni->data_in(data_in_ni);
		ni->credit_o(credit_o_ni);
		ni->hold(plasma_hold);
		ni->send_av(ni_send_av);
		ni->read_av(ni_read_av);
		ni->intr(ni_intr);
		ni->send_data(ni_send_data);
		ni->read_data(ni_read_data);
		ni->data_write(ni_data_write);
		ni->data_read(ni_data_read);
		ni->config(ni_config);
		
		dma_mas = new dma_master ("dma_mas");
		dma_mas->clock(clock);
		dma_mas->reset(reset);
		dma_mas->read_av(ni_read_av);
		dma_mas->read_data(dma_read_data);
		dma_mas->send_av(ni_send_av);
		dma_mas->send_data(dma_send_data);
		dma_mas->set_address(cpu_set_address);
		dma_mas->set_size(cpu_set_size);
		dma_mas->set_op(cpu_set_op);
		dma_mas->start(cpu_start);
		dma_mas->data_read(dma_data_read);
		dma_mas->data_write(dma_data_write);
		dma_mas->active(dma_active_sig);
		dma_mas->intr(dma_intr);
		dma_mas->intr_ack(cpu_ack);
		dma_mas->data_valid(data_valid);
		dma_mas->mem_address(dma_mem_address);
		dma_mas->mem_addr_ddr(dma_mem_addr_ddr);
		dma_mas->mem_ddr_read_req(dma_mem_ddr_read_req);
		dma_mas->mem_ddr_access(mem_ddr_access);
		dma_mas->mem_data_write(dma_mem_data_write);
		dma_mas->mem_data_read(dma_mem_data_read);
		dma_mas->mem_byte_we(dma_mem_write_byte_enable);
		
		repo_access = new access_repository ("repo_access");
		repo_access->clock(clock);
		repo_access->reset(reset);
		repo_access->read_req(read_req);
		repo_access->address(address);
		repo_access->data_valid(data_valid);
		repo_access->data_read(data_read);
		repo_access->mem_ddr_access(mem_ddr_access);
		repo_access->dma_mem_addr_ddr(dma_mem_addr_ddr);
		repo_access->dma_mem_ddr_read_req(dma_mem_ddr_read_req);
		repo_access->cpu_mem_address(cpu_mem_address);
		repo_access->cpu_mem_address_reg(cpu_mem_address_reg);
		repo_access->mem_hold(mem_hold);
		repo_access->data_read_reg(data_read_reg);
		
		router = new router_cc("router",router_address);
		router->clock(clock);
		router->reset_n(reset_n);
		router->clock_tx[EAST] (clock_tx[EAST] );
		router->clock_tx[WEST] (clock_tx[WEST] );
		router->clock_tx[NORTH](clock_tx[NORTH]);
		router->clock_tx[SOUTH](clock_tx[SOUTH]);
		router->clock_tx[LOCAL](clock_rx_ni);
		router->tx[EAST](tx[EAST]);
		router->tx[WEST](tx[WEST]);
		router->tx[NORTH](tx[NORTH]);
		router->tx[SOUTH](tx[SOUTH]);
		router->tx[LOCAL](rx_ni);
		router->credit_o[EAST](credit_o[EAST]);
		router->credit_o[WEST](credit_o[WEST]);
		router->credit_o[NORTH](credit_o[NORTH]);
		router->credit_o[SOUTH](credit_o[SOUTH]);
		router->credit_o[LOCAL](credit_i_ni);
		router->data_out[EAST](data_out[EAST]);
		router->data_out[WEST](data_out[WEST]);
		router->data_out[NORTH](data_out[NORTH]);
		router->data_out[SOUTH](data_out[SOUTH]);
		router->data_out[LOCAL](data_in_ni);
		router->rx[EAST](rx[EAST]);
		router->rx[WEST](rx[WEST]);
		router->rx[NORTH](rx[NORTH]);
		router->rx[SOUTH](rx[SOUTH]);
		router->rx[LOCAL](tx_ni);
		router->credit_i[EAST](credit_i[EAST]);
		router->credit_i[WEST](credit_i[WEST]);
		router->credit_i[NORTH](credit_i[NORTH]);
		router->credit_i[SOUTH](credit_i[SOUTH]);
		router->credit_i[LOCAL](credit_o_ni);
		router->data_in[EAST](data_in[EAST]);
		router->data_in[WEST](data_in[WEST]);
		router->data_in[NORTH](data_in[NORTH]);
		router->data_in[SOUTH](data_in[SOUTH]);
		router->data_in[LOCAL](data_out_ni);
		router->clock_rx[EAST] (clock_rx[EAST]);
		router->clock_rx[WEST] (clock_rx[WEST]);
		router->clock_rx[NORTH](clock_rx[NORTH]);
		router->clock_rx[SOUTH](clock_rx[SOUTH]);
		router->clock_rx[LOCAL](clock_tx_ni);
		
		SC_METHOD(reset_n_attr);
		sensitive << reset;
		
		SC_METHOD(sequential_attr);
		sensitive << clock.pos()  << reset.pos();
		
		SC_METHOD(comb_assignments);
		sensitive << cpu_mem_address << dma_mem_address << cpu_mem_address_reg << write_enable;
		sensitive << cpu_mem_data_write_reg << data_read_reg << irq_mask_reg << irq_status;
		sensitive << time_slice << ni_read_av << ni_send_av << ni_data_read << ni_config << tick_counter;
		sensitive << dma_active_sig << empty_out << data_read_ram << plasma_hold << debug_busy << mem_hold;
		sensitive << dma_send_data << cpu_send_data << dma_read_data << cpu_read_data << dma_data_write;
		sensitive << cpu_set_op << cpu_set_size << cpu_set_address << dma_enable_internal_ram;
		sensitive << mem_data_read << cpu_enable_internal_ram << cpu_mem_write_byte_enable_reg << dma_mem_write_byte_enable;
		sensitive << dma_mem_data_write << ni_intr << dma_intr << debug_write_busy;
		
		SC_METHOD(mem_mapped_registers);
		sensitive << cpu_mem_address_reg;
		sensitive << ni_read_av;
		sensitive << ni_send_av;
		sensitive << ni_data_read;
		sensitive << ni_config;
		sensitive << tick_counter;
		sensitive << empty_out;
		sensitive << data_read_ram;
		sensitive << time_slice;
		sensitive << irq_status;

        SC_METHOD(end_of_simulation);
		sensitive << end_sim_reg;
	}
	private:
		regaddress router_address;
};
#endif
