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
-- Deploy this file to wireshark's lua folder. There are two possible folders, one
-- for root users (required if you invoke wireshark using sudo), and another one 
-- for users. You can find these information in Help->About->Folder, in wireshark gui.

-- In my case, I deployed this file to /usr/lib/x86_64-linux-gnu/wireshark/plugins/, 
-- and it worked well.

orca_proto = Proto("orca", "ORCA MPSoC network protocol")

-- fields
--message_length = ProtoField.int32("orca.hermes.length", "length_flit", base.DEC)

--orca_proto.fields = { message_length }


-- dissector
function orca_proto.dissector(buffer, pinfo, tree)

	pinfo.cols.protocol = orca_proto.name

	local subtree = tree:add(orca_proto, buffer(), "Hermes NoC Protocol")
	
	-- address flit
	local addrflit = subtree:add(buffer(0,2), "Hermes Address Flit: " .. buffer(0,2):uint())
	addrflit:add(buffer(0,1), "Hermes Address" .. 
		"  X=" .. bit.rshift(bit.tobit(buffer(0,1):uint()), 4) .. 
		", Y=" .. bit.band(bit.tobit(buffer(0,1):uint()), 15))
	addrflit:add(buffer(1,1), "Hermes Address Unused: " .. buffer(0,2):uint())

	--length flit
	subtree:add(buffer(2,2), "Hermes Size Flit: " .. bit.bswap(bit.tobit(buffer(2,2):uint())))

	-- payload
	subtree = subtree:add(buffer(4,124), "Hermes Payload Flits: " .. buffer(2,2):uint())
	
	-- hfos proto
	subtree:add(buffer(4,2), "HellfireOS Destination Port: " .. buffer(2,2):uint())
	subtree:add(buffer(6,2), "HellfireOS Destination Port: " .. buffer(2,2):uint())
	subtree:add(buffer(8,2), "HellfireOS Destination Port: " .. buffer(2,2):uint())
	subtree:add(buffer(10,2), "HellfireOS Destination Port: " .. buffer(2,2):uint())
	subtree:add(buffer(12,2), "HellfireOS Destination Port: " .. buffer(2,2):uint())
	
	-- payload
	subtree = subtree:add(buffer(12,2), "HellfireOS Payload (application data): " .. buffer(2,2):uint())
	
	-- subtree:add_le(message_length, buffer(0,4))

--  subtree = subtree:add(buffer(2,2), "next two bytes")
--  subtree:add(buffer(2,1), "3rd byte: " .. buffer(2,1):uint())
--  subtree:add(buffer(3,1), "4th byte: " .. buffer(3,1):uint())
end

-- hook
local udp_port = DissectorTable.get("udp.port")
udp_port:add(9999, orca_proto)
udp_port:add(8888, orca_proto)


