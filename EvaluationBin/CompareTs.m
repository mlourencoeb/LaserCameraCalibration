function [norm_t, theta_R] = CompareTransformations(Tref, Test)

	t_ref = Tref(1:3, 4);
	t_est = Test(1:3, 4);
	norm_t = norm(t_ref - t_est);
	
	Rref = Tref(1:3, 1:3);
	Rest = Test(1:3, 1:3);
	theta_R = norm(rodrigues(transpose(Rest)*Rref))*180/pi;
    
end