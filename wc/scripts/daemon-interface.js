var SERVER_HANDLER = "http://localhost/ipt/handler.php";


/** sends a message to daemon requesting the 
  * value of some metric.
  * params:
  *		obj_name: container to hold response data
  *		param_name: name of the parameters to be read */
function daemon_get(param_name, obj_name){
		
	var query = "?param_name=" + param_name;
	query = SERVER_HANDLER + query;
		
	//send ajax request to server handler and write 
	//response to the given object
	$.ajax({url: query, success: function(result){
        $(obj_name).html(result);
    }});
}

/**
 * sends a message to daemon for setting
 * a given parameter to some value. 
 * params:
 *		param_name: name of the parameter to be set
 *		value: self-explanatory
 *		obj_name: container to hold response data
 * remark:
 *		server side values are treate as uint32_t and
 *		parameters of string type must respect the limit
 *		of 32-bit. */
function daemon_set(param_name, param_value, obj_name){

	var query = "?param_name=" + param_name + "&param_value=" + param_value;
	query = SERVER_HANDLER + query;
	
	//send ajax request to server handler and write 
	//response to the given object
	$.ajax({url: query, success: function(result){
        $(obj_name).html(result);
    }});
}