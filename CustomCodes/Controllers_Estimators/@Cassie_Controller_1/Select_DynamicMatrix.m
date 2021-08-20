function [B_bar,B_bar_inv,H_bar,M_bar,M_bar_inv] = Select_DynamicMatrix(obj,DynamicMatrixLibrary,left_knee,right_knee)
            if obj.stanceLeg == -1
                phi = clamp(left_knee, DynamicMatrixLibrary.Left.knee(1), DynamicMatrixLibrary.Left.knee(end));
                B_bar = interp1(DynamicMatrixLibrary.Left.knee, DynamicMatrixLibrary.Left.B_bar,phi);
                B_bar_inv = interp1(DynamicMatrixLibrary.Left.knee, DynamicMatrixLibrary.Left.B_bar_inv,phi);
                H_bar = interp1(DynamicMatrixLibrary.Left.knee, DynamicMatrixLibrary.Left.H_bar,phi);
                M_bar = interp1(DynamicMatrixLibrary.Left.knee, DynamicMatrixLibrary.Left.M_bar,phi);
                M_bar_inv = interp1(DynamicMatrixLibrary.Left.knee, DynamicMatrixLibrary.Left.M_bar_inv,phi);
            else
                phi = clamp(right_knee, DynamicMatrixLibrary.Right.knee(1), DynamicMatrixLibrary.Right.knee(end));
                B_bar = interp1(DynamicMatrixLibrary.Right.knee, DynamicMatrixLibrary.Right.B_bar,phi);
                B_bar_inv = interp1(DynamicMatrixLibrary.Right.knee, DynamicMatrixLibrary.Right.B_bar_inv,phi);
                H_bar = interp1(DynamicMatrixLibrary.Right.knee, DynamicMatrixLibrary.Right.H_bar,phi);
                M_bar = interp1(DynamicMatrixLibrary.Right.knee, DynamicMatrixLibrary.Right.M_bar,phi);
                M_bar_inv = interp1(DynamicMatrixLibrary.Right.knee, DynamicMatrixLibrary.Right.M_bar_inv,phi);
            end
            B_bar = reshape(B_bar,size(B_bar,2),size(B_bar,3));
            B_bar_inv = reshape(B_bar_inv,size(B_bar_inv,2),size(B_bar_inv,3));
            H_bar = reshape(H_bar,size(H_bar,2),size(H_bar,3));
            M_bar = reshape(M_bar,size(M_bar,2),size(M_bar,3));
            M_bar_inv = reshape(M_bar_inv,size(M_bar_inv,2),size(M_bar_inv,3));
end

