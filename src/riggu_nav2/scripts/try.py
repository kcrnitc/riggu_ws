#    def update_pid_loop():
        
#         c_time = time.time_ns()
#         dt = (c_time-prev_time)*10e-9

#         t_vr = self.t_vr
#         t_vl = self.t_vl

#         current_l_tick_count = self.tick_count.x
#         current_r_tick_count = self.tick_count.y

#         c_vl = (current_l_tick_count - prev_l_tick_count)/dt
#         c_vr = (current_r_tick_count - prev_r_tick_count)/dt
#         print('dt: ',dt)
#         print("Left_vel : ",c_vl)
#         print("Right_vel : ",c_vr)
#         c_vl = c_vl/self.cpr*self.wheel_dia*3.14
#         c_vr = c_vr/self.cpr*self.wheel_dia*3.14
#         self.cvl_pub.publish(Float32(data=float(c_vl)))
#         self.tvl_pub.publish(Float32(data=float(t_vl)))
#         self.tvr_pub.publish(Float32(data=float(t_vr)))
#         self.cvr_pub.publish(Float32(data=float(c_vr)))


#         el = c_vl-t_vl
#         er = c_vr-t_vr

#         pidl.Kd = self.kd
#         pidl.Ki = self.ki
#         pidl.Kp = self.kp

#         pidr.Kd = self.kd
#         pidr.Ki = self.ki
#         pidr.Kp = self.kp

#         pidl.setpoint = self.t_vl
#         pidr.setpoint = self.t_vr

#         ctrl_effort_left = pidl(el)
#         ctrl_effort_right = pidr(er)

#         pwm_l.data = float(ctrl_effort_left)
#         pwm_r.data = float(ctrl_effort_right)
        
#         print("params :",self.kp,self.ki,self.kd)

#         print("Left : ",ctrl_effort_left)
#         print("Right : ",ctrl_effort_right)

#         self.left_pub.publish(pwm_l)
#         self.right_pub.publish(pwm_r)
#         prev_l_tick_count = current_l_tick_count
#         prev_r_tick_count = current_r_tick_count
#         prev_time = c_time