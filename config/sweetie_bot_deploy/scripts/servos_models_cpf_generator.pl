#!/usr/bin/perl

use YAML;

# generate cpf servo entities
sub insert_servo_model {
	my ($prop_name, $servo_name, $params_ref)= @_; # references to array and hash
	my $alpha_ref = $params_ref->{'alpha'};
	print <<"ENDHEREDOC";
    <struct name="$prop_name" type="/sweetie_bot_servo_model_msg/ServoModel">
      <simple name="name" type="string"><value>$servo_name</value></simple>
      <simple name="kp" type="double"><value>$params_ref->{'kp'}</value></simple>
      <simple name="kgear" type="double"><value>$params_ref->{'kgear'}</value></simple>
      <struct name="alpha" type="double">
        <simple name="Element0" type="double"><value>$alpha_ref->[0]</value></simple>
        <simple name="Element1" type="double"><value>$alpha_ref->[1]</value></simple>
        <simple name="Element2" type="double"><value>$alpha_ref->[2]</value></simple>
        <simple name="Element3" type="double"><value>$alpha_ref->[3]</value></simple>
      </struct>
    </struct>
ENDHEREDOC
}


# parse input in form of YAML file
my @lines = <stdin>;
my $hashref = Load( join('', @lines) );

# print cpf header
print <<"ENDHEREDOC";
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE properties SYSTEM "cpf.dtd">
<properties>
ENDHEREDOC

# default servo model
insert_servo_model('default_servo_model', 'default', $hashref->{'default'} );

# servo model groups
print <<"ENDHEREDOC";
  <struct name="servo_models" type="/sweetie_bot_servo_model_msg/ServoModel[]">
ENDHEREDOC
# element counter
my $count = 0;
foreach $group_ref (@{$hashref->{'servos'}}) {
	# insert servo model
	foreach $servo (@{$group_ref->{'names'}}) {
		insert_servo_model('Element' . $count++, $servo, $group_ref->{'model'});
	}
}
print <<"ENDHEREDOC";
  </struct>
ENDHEREDOC

print <<"ENDHEREDOC";
</properties>
ENDHEREDOC
