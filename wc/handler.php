<?php
ini_set("error_reporting", E_ALL);
ini_set("display_errors", true);

#=========================== CONFIG ===

/**
 * Internet-layer Protocol to be used when connecting to ipt
 * server. Also called the domain of the sock.
 * AF_INET  = IPv4
 * AF_INET6 = IPv6 */
$domain = AF_INET;

/**
 * Type of socket to be used when connecting to ipt
 * server. Note that TCP authentication may not be
 * implemented yet.
 * SOCK_STREAM is necessary for TCP
 * SOCK_DGRAM  is necessary for UDP */
$type = SOCK_DGRAM;

/**
 * Transport-layer protocol to be used to connect
 * to ipt server. UPD is default since we assume 
 * that both ipd-daemon and web-client will be 
 * deployed to the same server, i.e., communication
 * is made through local link, which should not
 * fail, ever. */
$proto = SOL_UDP;

#=======================================

function make_header($param_name, $param_value, $mode){

}

$param_name  = null;
$param_value = null;
$mode = "NONE";

//check whether param_name has been passed
if(isset($_GET['param_name'])){
	$param_name  = $_GET["param_name"];
	$mode = "GET";	
}else{
	die("param_name is not set.");
}

//check whether param_value has been passed
if(isset($_GET['param_value'])){
	$param_value = $_GET["param_value"];
	$mode = "SET";
}

//make header
$header = make_header($param_name, $param_value, $mode);

//open socket
$socket = socket_create($domain, $type, $proto);

if ($socket === false) {
    die("socket_create() failed: reason: " . socket_strerror(socket_last_error()) . "\n");
}

//Bind to localhost at port 55554.
if(!socket_bind($socket, 'localhost', 55554))
    die("socket_bind failed.\n");

//

socket_close($socket);

echo $param_name;

?>