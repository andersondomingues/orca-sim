-- 
-- This file is part of project URSA. More information on the project
-- can be found at 
--
-- URSA's repository at GitHub: http://https://github.com/andersondomingues/ursa
--
-- Copyright (C) 2018-2019 Anderson Domingues, <ti.andersondomingues@gmail.com>
--
-- Description:
-- PROTOCOL DISSECTOR FOR URSA+HF PROTOCOL
-- This file constains the implementation of a dissector, 
-- that is, a plugin for Wireshark to friendly display 
-- data from the payload of transport protocols. In this file
-- we register the dissector to hook to UDP ports 8888 and 9999

-- How to use
-- Deploy this file to wireshark's plugins folder. There are two possible folders, one
-- for root users (required if you invoke wireshark using sudo), and another one 
-- for users. You can find these information in Help->About->Folder, in wireshark gui.

-- In my case, I deployed this file to /usr/lib/x86_64-linux-gnu/wireshark/plugins/, 
-- and it worked well.

-- Name your protocol here. The first string corresponds to the name
-- used in wireshark's filter, so you can display only the packets 
-- you are interested in. The second string is a more comprehensive 
-- description, displayed in several other field in wireshark.
orca_proto = Proto("orca", "ORCA MPSoC network protocol")

-- Begin of the dissector. Dissectors are functions, which receive
-- buffer (the actual packet data), protocol information (pinfo), and
-- a tree of fields tree) as parameters. These functions are called by
-- wireshark when you register them to some listener. Listerners are 
-- show in the botton of this file.
function orca_proto.dissector(buffer, pinfo, tree)
	
	-- Set the name displayed in "Protocol" column in wireshark gui
	pinfo.cols.protocol = orca_proto.name

	-- Add a subtree to the protocol hierarchy for our protocol
	local subtree = tree:add(orca_proto, buffer(), "ORCA MPSoC Protocol")
	
	-- Add the first flit (address flit) from hermes proto as the first 
	-- field in our protocol and treat data to display X and Y coordinates
	-- of the target cpu at the side of the field name
	local addrflit = subtree:add(buffer(0,2), "Hermes Address Flit: " .. buffer(0,2):uint())
	addrflit:add(buffer(0,1), "Hermes Target CPU" .. 
		"  X=" .. bit.rshift(bit.tobit(buffer(0,1):uint()), 4) .. 
		", Y=" .. bit.band(bit.tobit(buffer(0,1):uint()), 15))

	-- Add a dummy field to indicate that some bytes are unused within address field
	addrflit:add(buffer(1,1), "Hermes Address Unused: " .. buffer(0,2):uint())
	
	-- Add the size field (payload in HellfireOS, size flit in Hermes) to our protocol
	local sizeflit = bit.tobit(buffer(2,2):int())
	subtree:add(buffer(2,2), "Hermes Size Flit: " .. sizeflit)

	-- Add the remaining flits of HellfireOS protocol
	subtree = subtree:add(buffer(4,124), "Hermes Payload Flits (HellfireOS Protocol): " .. buffer(2,2):uint())
	
	-- HFOS
	subtree:add(buffer(4,2), "HellfireOS Source CPU: " .. buffer(2,2):uint())
	subtree:add(buffer(6,2), "HellfireOS Source Port: " .. buffer(2,2):uint())
	subtree:add(buffer(8,2), "HellfireOS Target Port: " .. buffer(2,2):uint())
	subtree:add(buffer(10,2), "HellfireOS Message Size: " .. buffer(2,2):uint())
	subtree:add(buffer(12,2), "HellfireOS Sequence Number: " .. buffer(2,2):uint())
	subtree:add(buffer(14,2), "HellfireOS Channel: " .. buffer(2,2):uint())
	
	-- payload
	subtree = subtree:add(buffer(16,112), "HellfireOS Payload (application data): " .. buffer(2,2):uint())
end

-- Hook the dissector to UDP listerner. Here, we configure our protocol
-- to append to the payload of UDP packets if their port corresponds to
-- 8888 or 9999.
local udp_port = DissectorTable.get("udp.port")
udp_port:add(9999, orca_proto)
udp_port:add(8888, orca_proto)


