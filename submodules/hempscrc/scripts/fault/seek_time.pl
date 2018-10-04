	#!/usr/bin/perl

	if ($ARGV[0] ne "") {
		$log_file=shift; #arquivo com log		
	}
	else {
	  print "ERRO, falta de argumento\n";
	  exit;
	}

	open( C_FILE, "<$log_file" );
	my @c_lines = <C_FILE>;
	close(C_FILE);

	my @time;
	my @service;

	my $is_faulty = 0;
	$i=0;
	print "==========================================================================\n";
	foreach $c_line (@c_lines) {
		if($c_line =~ /(\d*):D/){
			# print "DEL:$1\n";
			$service[$i] = $1;
			if($is_faulty == 0){
				$t[0]=$1;
			}
			else{
				$t[4]=$1;
				$temp=$t[4]-$t[3];
				# print "t4-t3=$temp\n";
				print "$temp\t";
			}
		}
		elsif($c_line =~ /(\d*):U/){
			# print "UNR:$1\n";
			$service[$i] = $1;
			$t[1]=$1;
			$temp=$t[1]-$t[0];
			# print "t1-t0=$temp\n";
			print "$temp\t";
			$is_faulty = 1;
		}
		elsif($c_line =~ /(\d*):B/){
			# print "BAC:$1\n";
			$service[$i] = $1;
			$t[3]=$1;
			$temp=$t[3]-$t[1];
			# print "t3-t1=$temp\n";
			print "$temp\t";
		}
		elsif($c_line =~ /(\d*):A/){
			$delack[$i]=$1-$t[0];
			$i++;
			if($is_faulty == 1){
				# print "ACK:$1\n";
				$service[$i] = $1;

				$t[5]=$1;
				$temp=$t[5]-$t[4];
				# print "t5-t4=$temp\n";
				print "$temp\t";
				$temp=$t[5]-$t[0];
				print "total overhead:$temp\n";
				$is_faulty = 0;
			}
		}
	}

	# print "==========================================================================\n";
	# my $t_seek_unr;
	# my $t_seek;
	# my $t_backtrack;


	# $again = 0;

	for($i=0;$i<8;$i++){
		print "$delack[$i]\n";
	}
	# 	#seek unreachable start and finish times
	# 	if($service[$i] =~ /:DEL/){
	# 		# print "fail_detection: $time[$i] ns\n";
	# 		$t_seek_unr = $time[$i];
	# 	}
	# 	elsif(($service[$i] =~ /UNREACHABLE/) && ($again == 0)){#do only for the first UNREACHABLE
	# 		$again = 1;
	# 		# print "SEEK_UNREACHABLE: $time[$i] ns\n";
	# 		$t_seek_unr = $time[$i]-$t_seek_unr;
	# 		# print "t_seek_unr: $t_seek_unr ns\n";
	# 		$t_seek = $time[$i];
	# 	}

	# 	#normal seek start and finish times
	# 	if($service[$i] =~ /seek from/){
	# 		# print "seek: $time[$i] ns\n";
	# 	}
	# 	elsif($service[$i] =~ /SEEK/){
	# 		# print "SEEK FOUND A PATH: $time[$i] ns\n";
	# 		$t_seek = $time[$i] - $t_seek;
	# 		$t_backtrack = $time[$i];
	# 		print "t_seek: $t_seek ns\n";
	# 	}

	# 	#backtrack finish time
	# 	if($service[$i] =~ /BACKTRACK/){
	# 		$t_backtrack = $time[$i] - $t_backtrack;
	# 		print "t_backtrack: $t_backtrack ns\n"
	# 	}

	# }