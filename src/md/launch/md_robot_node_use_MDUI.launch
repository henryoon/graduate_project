<launch>
    <node pkg="md" type="md_robot_node" name="md_robot_node" output="screen">
        <param name = "use_MDUI"                 value = "1"/>      <!-- 0: not use MDUI, 1: use MDUI -->
        <param name = "serial_port"              value = "ttyUSB1"/> <!-- COM port -->
        <param name = "serial_baudrate"          value = "57600"/>   <!-- MDUI baudrate -->
        <param name = "wheel_radius"             value = "0.0935"/> <!-- unit: meter -->
        <param name = "wheel_length"             value = "0.454"/>  <!-- unit: meter -->
        <param name = "reduction"                value = "30"/>
        <param name = "reverse_direction"        value = "0"/>      <!-- 0: forward, 1: reverse -->
        <param name = "maxrpm"                   value = "1000"/>   <!-- unit: RPM -->
        <param name = "enable_encoder"           value = "0"/>      <!-- 0: hall sensor, 1: encoder, Use the encoder only when using an in-wheel motor -->
        <param name = "encoder_PPR"              value = "900"/>    <!-- if use encoder position, encoder PPR, reference PID 126(PID_PPR) -->
        <param name = "slow_start"               value = "300"/>    <!-- unit: RPM -->
        <param name = "slow_down"                value = "300"/>    <!-- unit: RPM -->
    </node>
</launch>
