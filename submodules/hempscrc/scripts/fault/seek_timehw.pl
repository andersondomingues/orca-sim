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


	foreach $c_line (@c_lines) {
		if($c_line =~ s/.*Note: //g){#removes everything before Note:
			# print $c_line;
			$service[$i] = $c_line;
		}
		elsif($c_line =~ /.{8}(\d*) ns/){#gets the number between the 8th caracter and " ns"
			$time[$i] = $1;
			# print "$time[$i]: $service[$i]";
			$i++;
			# next;
		}
	}

	print "==========================================================================\n";
	my $t_seek_unr;
	my $t_seek;
	my $t_backtrack;


	$again = 0;

	for($i=0;$i<@service;$i++){

		#seek unreachable start and finish times
		if($service[$i] =~ /fail detected/){
			print "fail_detection: $time[$i] ns\n";
			$t_seek_unr = $time[$i];
		}
		elsif(($service[$i] =~ /UNREACHABLE/) && ($again == 0)){#do only for the first UNREACHABLE
			$again = 1;
			print "SEEK_UNREACHABLE: $time[$i] ns\n";
			$t_seek_unr = $time[$i]-$t_seek_unr;
			print "t_seek_unr: $t_seek_unr ns\n";
			$t_seek = $time[$i];
		}

		#normal seek start and finish times
		if($service[$i] =~ /seek from/){
			# print "seek: $time[$i] ns\n";
		}
		elsif($service[$i] =~ /SEEK/){
			# print "SEEK FOUND A PATH: $time[$i] ns\n";
			$t_seek = $time[$i] - $t_seek;
			$t_backtrack = $time[$i];
			# print "t_seek: $t_seek ns\n";
		}

		#backtrack finish time
		if($service[$i] =~ /BACKTRACK/){
			$t_backtrack = $time[$i] - $t_backtrack;
			# print "t_backtrack: $t_backtrack ns\n"
		}

	}