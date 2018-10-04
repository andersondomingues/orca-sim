#!/usr/bin/perl

use Switch;


foreach $i (0 .. $#ARGV) {
    $myString = $ARGV[$i];
    my @char_array = split(//,"$myString");
    foreach my $char (@char_array){
        print $char;
    }
    print "\n";
    
    #$i=0;#position in the source header
    #foreach my $char (@char_array){
        #switch($i){
            #case 0         { print "-"; $i++;}
            #case 1         { print $char; $i++;}
            #case 2         { print $char; $i++;}
            #case 3         { print $char; $i=0;}
            #case default   { print "-"}
        #}
    #}
    #print "\n";
    
    $i=0;#position in the source header
    foreach my $char (@char_array){
        switch($i){
            case 0         { print ""; $i++;}
            case 1         { print_port($char); $i++;}
            case 2         { print_port($char); $i++;}
            case 3         { print_port($char); $i=0;}
            case default   { print "-"}
        }
    }
    print "\n";
    
    $i=0;#position in the source header
    foreach my $char (@char_array){
        switch($i){
            case 0         { print ""; $i++;}
            case 1         { print_channel($char); $i++;}
            case 2         { print_channel($char); $i++;}
            case 3         { print_channel($char); $i=0;}
            case default   { print "-"}
        }
    }
    print "\n";
    #$i=$i+1;
}

sub print_port{
    $char = $_[0];
    switch ($char){
        case "0"        { print "E"}
        case "1"        { print "W"}
        case '2'        { print "N"}
        case '3'        { print "S"}
        case '4'        { print "E"}
        case '5'        { print "W"}
        case '6'        { print "N"}
        case '7'        { print "S"}
        case '8'        { print "-"}
        case '9'        { print "-"}
        case ["a","A"]  { print "-"}
        case ["b","B"]  { print "-"}
        case ["c","C"]  { print "-"}
        case ["d","D"]  { print "-"}
        case ["e","E"]  { print "-"}
        case ["f","F"]  { print "-"}
        case default    { print ""}
    }
}
sub print_channel{
    $char = $_[0];
    switch ($char){
        case "0"        { print "0"}
        case "1"        { print "0"}
        case '2'        { print "0"}
        case '3'        { print "0"}
        case '4'        { print "1"}
        case '5'        { print "1"}
        case '6'        { print "1"}
        case '7'        { print "1"}
        case '8'        { print "-"}
        case '9'        { print "-"}
        case ["a","A"]  { print "-"}
        case ["b","B"]  { print "-"}
        case ["c","C"]  { print "-"}
        case ["d","D"]  { print "-"}
        case ["e","E"]  { print "-"}
        case ["f","F"]  { print "-"}
        case default    { print ""}
    }
}
