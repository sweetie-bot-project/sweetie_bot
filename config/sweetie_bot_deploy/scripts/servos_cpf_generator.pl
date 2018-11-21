#!/usr/bin/perl
print(
"<?xml version=\"1.0\" encoding=\"UTF-8\"?>
<!DOCTYPE properties SYSTEM \"cpf.dtd\">
<properties>
  <struct name=\"servos\" type=\"PropertyBag\">
    <description>Herkulex servo description.</description>
	<struct name=\"broadcast\" type=\"PropertyBag\">
		<description>Broadcast ID.</description>
		<simple name=\"servo_id\" type=\"ulong\">
			<description>Servo hardware ID.</description>
			<value>254</value>
		</simple>
		<simple name=\"offset\" type=\"ulong\">
			<description>Servo zero position offset (raw).</description>
			<value>512</value>
		</simple>
		<simple name=\"reverse\" type=\"boolean\">
			<description>Reverse direction flag.</description>
			<value>0</value>
		</simple>
		<struct name=\"registers_init\" type=\"PropertyBag\">
			<description>These values are written to RAM register of ALL servos after reboot. The servo-specific values override them.</description>
			<simple name=\"torque_control\" type=\"ulong\">
				<description>Servo opertional state: ON=96, BRAKE=64, FREE=0</description>
				<value>96</value>
			</simple>
		</struct>
	</struct>\n");

while (<>) {
    my ($joint, $ID, $reverse, $offset) = split(/\s+/, $_);
    print(
"    <struct name=\"$joint\" type=\"PropertyBag\">
        <description>Servo description.</description>
        <simple name=\"servo_id\" type=\"ulong\">
            <description>Servo hardware ID.</description>
            <value>$ID</value>
        </simple>
        <simple name=\"offset\" type=\"ulong\">
            <description>Servo zero position offset (raw).</description>
            <value>$offset</value>
        </simple>
        <simple name=\"reverse\" type=\"boolean\">
            <description>Reverse direction flag.</description>
            <value>$reverse</value>
        </simple>
    </struct>\n");
}

print(
"  </struct>
</properties>\n");
