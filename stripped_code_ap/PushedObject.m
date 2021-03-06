classdef PushedObject < matlab.mixin.Copyable
    % Class of object being pushed. 
   properties
      % Limit surface related.
      ls_coeffs
      ls_type = 'quadratic'
      ls_coeffs_cp
      % Pressure related.
      support_pts  %2*N
      pressure_weights  % N*1
      % Coefficient of contact friction between pusher and the object.
      %mu_contact
      % Shape and geometry related. 
      shape_id % unique id 
      shape_type % type for different methods of computing distance.
      shape_vertices % object coordinate frame. 2*N.
      shape_parameters % radius of circle, two axis length of ellipse, etc.
      pho % radius of gyration.
      nsides_symmetry % specify the symmetry order (if any).
      noise_df % Specify the degree of freedom in wishart sampling. Bigger value is smaller noise.
      % Pose related. object coordinate frame w.r.t the world frame.
      pose %3*1: [x;y;theta]
      %cur_shape_vertices % shape vertices in world frame.
      
      % Configuration parameters for Sampling of wrench,twist pairs. 
      % When the user only gives pressure points and weights for constructor 
      % function, we will sample wrench and twist pairs instead. 
      % Total number of center of rotations (CORs). 
      num_cors
      % The ratio [0,1] of CORs that will be sampled on the limit surface facet. 
      r_facet 
      
   end
   methods
       function obj = PushedObject(support_pts, pressure_weights, shape_info, ls_type, ls_coeffs, nsides_symmetry) 
            obj.support_pts = support_pts;
            obj.pressure_weights = pressure_weights;
            obj.shape_id = shape_info.shape_id;
            obj.shape_type = shape_info.shape_type;
            obj.pho = shape_info.pho;
            % The default symmetry order is 1.
            obj.nsides_symmetry = 1;
            obj.noise_df = 50;
            if strcmp(obj.shape_type,'polygon')
                obj.shape_vertices = shape_info.shape_vertices;
            else
                obj.shape_parameters = shape_info.shape_parameters;
            end
            if (nargin >= 5)
                obj.ls_type = ls_type;
                obj.ls_coeffs = ls_coeffs;
                obj.ls_coeffs_cp = obj.ls_coeffs;
            end
            if (nargin == 6)
                obj.nsides_symmetry = nsides_symmetry;
            end
       end
        
       function [obj] = FitLS(obj,ls_type, num_cors, r_facet, flag_plot)
            if (nargin < 3)
                num_cors = 200;
            end
            if (nargin < 4)
                r_facet = 0.4;  
            end
            if (nargin < 5)
                flag_plot = 0;
            end
            obj.SetWrenchTwistSamplingConfig(num_cors, r_facet);
            obj.FitLSFromPressurePoints(ls_type, flag_plot);
       end
       
       % For now, the noise injection is only for ellipsoid/quadratic model.
       function [] = InjectLSNoise(obj)
           strcmp(obj.ls_type, 'quadratic')
           % Larger df is smaller deviation.
           df = obj.noise_df; 
           obj.ls_coeffs = wishrnd(obj.ls_coeffs_cp,df)/df;
       end
       
       function [obj] = SetWrenchTwistSamplingConfig(obj, num_cors, r_facet)
           if (num_cors < 15)
                disp('More wrench-twist points would be better.')
           end
           obj.num_cors = num_cors;
           obj.r_facet = r_facet;
       end
       
       function [obj] = FitLSFromPressurePoints(obj, ls_type, flag_plot)
            if (nargin == 1)
                obj.ls_type = 'quadratic';
            else
                if (strcmp(ls_type, 'quadratic') || strcmp(ls_type, 'poly4'))
                    obj.ls_type = ls_type;
                else
                    disp('limit surface type not recognized %s\n', ls_type);
                end
            end
            if (nargin < 3)
                flag_plot = 0;
            end
            % Generate random points. 
            num_pts = size(obj.support_pts, 2);
            num_facet_pts = ceil(obj.r_facet * (obj.num_cors / 2) / num_pts);
            num_other_pts = ceil((1 - obj.r_facet) * (obj.num_cors / 2));
            CORs = GenerateRandomCORs3(obj.support_pts, num_other_pts, num_facet_pts);
            [F, V] = GenFVPairsFromPD(obj.support_pts, obj.pressure_weights, CORs);
            % Normalize the 3rd component of wrench and twist pairs by
            % characteristic length. Here F,V are all N*3, each row is a
            % data point. Twists are additionally normalized to unit
            % vector.
            [V, F] = NormalizeForceAndVelocities(V, F, obj.pho);
            [obj.ls_coeffs, xi, delta, pred_V_dir, s] = FitEllipsoidForceVelocityCVX(F', V', 1, 2, 1, flag_plot);
            obj.ls_coeffs_cp = obj.ls_coeffs;
       end
           
       function [pt_closest, dist] = FindClosestPointAndDistanceWorldFrame(obj, pt)
         % Input: pt is a 2*K column vector. 
         % Output: distance and the projected/closest point (2*K) on the object boundary.
         num_pts = size(pt, 2);
         dist = zeros(num_pts, 1); pt_closest = zeros(2, num_pts);
         theta = obj.pose(3);
         R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
         if strcmp(obj.shape_type, 'polygon')
             cur_shape = bsxfun(@plus, R * obj.shape_vertices, obj.pose(1:2));
             [tip_proj, dist] = projPointOnPolygon(pt', cur_shape');
             pt_closest = polygonPoint(cur_shape', tip_proj);
             pt_closest = pt_closest';
             
         elseif strcmp(obj.shape_type, 'circle')
             %dist = norm(pt - obj.pose(1:2));
             vec = bsxfun(@minus, pt, obj.pose(1:2));
             dist = sqrt(sum(vec.^2));
             pt_closest = obj.pose(1:2) + obj.shape_parameters.radius * bsxfun(@rdivide, vec, dist);
             dist = bsxfun(@minus, dist, obj.shape_parameters.radius);
             
         elseif strcmp(obj.shape_type, 'ellipse')
         else
            fprintf('Shape meta type not supported%s\n', obj.shape_type);
         end
      end
      
      function [vec_local] = GetVectorInLocalFrame(obj, vec) 
           % Input: column vectors 2*K in world frame.
           % Output: rotated to local frame.
           theta = obj.pose(3);
           R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
           vec_local = R' * vec;
      end
            
      function [flag_contact, pt_contact, vel_contact, outward_normal_contact] = ...
          GetRoundFingerContactInfo(obj, pt_finger_center, finger_radius, twist)
          % Input: pt_center (2*K): center of round fingers in world frame. 
          % finger_radius: radius of the round finger in meter.
          % global twist (3*K) ([vx,vy,omega]) of the finger body w.r.t world frame.
          % Output: flag_contact: whether any point of the round finger
          % will be in contact with the object.
          % pt_contact: the point (in world frame) that contacts the object.  
          % vel_contact: contact point linear velocity in world frame.
          % outward_normal: contact normal in world frame.
          num_fingers = size(pt_finger_center, 2);
          flag_contact = zeros(num_fingers, 1);
          pt_contact = zeros(2, num_fingers);
          vel_contact = zeros(2, num_fingers);
          outward_normal_contact = zeros(2, num_fingers);
 
          [pt_closest, dist] = obj.FindClosestPointAndDistanceWorldFrame(pt_finger_center); 
          r_blem = 1.00 + 1e-3;
          indices_contact = dist < (finger_radius * r_blem);
          flag_contact(indices_contact) = 1;
          pt_contact(:, indices_contact) = pt_closest(:, indices_contact);
          vel_contact(:, indices_contact) = twist(1:2, indices_contact) + bsxfun(@times, twist(3,indices_contact), [-pt_contact(2,indices_contact); pt_contact(1,indices_contact)]);
          outward_normal_contact(:, indices_contact) = pt_finger_center(:, indices_contact) - pt_contact(:, indices_contact);
          outward_normal_contact(:, indices_contact) = bsxfun(@rdivide, outward_normal_contact(:, indices_contact), sqrt(sum(outward_normal_contact(:, indices_contact).^2)) + eps);
      end
      
      function [min_dist] = FindClosestDistanceToHand(obj, hand)
          min_dist = 1e+9;
          if strcmp(obj.shape_type, 'polygon')
                [finger_twists, finger_poses] = hand.GetFingerGlobalTwistsAndCartesianWrtInertiaFrame();
                % Look at each polygonal or point finger's contact information. 
                for ind_finger = 1:1:hand.num_fingers
                    [dist_finger] = PolygonToPolygonDistance(...
                        hand.finger_geometries{ind_finger}, obj.shape_vertices, finger_poses(:, ind_finger), obj.pose);
                    if (dist_finger < min_dist)
                        min_dist = dist_finger;
                    end
                end
          end
      end
      
      function [flag_contact, pt_contacts, vel_contacts, outward_normal_contacts] = ...
              GetHandContactInfo(obj, hand)
      % Given the hand which contains, geometries, pose and twists in
      % global frame, compute whether the object is in contact with the
      % hand and if so find all contact points and velocities in world
      % frame.
      % Output flag_contact indicates if each finger is in contact.
      % all other contact informations are column vectors per each contact
      % point. 
      flag_contact = zeros(hand.num_fingers, 1);
      pt_contacts = [];
      vel_contacts = [];
      outward_normal_contacts = [];
       if strcmp(obj.shape_type, 'polygon')
           [finger_twists, finger_poses] = hand.GetFingerGlobalTwistsAndCartesianWrtInertiaFrame();
           % Look at each polygonal or point finger's contact information. 
           for ind_finger = 1:1:hand.num_fingers
               %display(ind_finger);
               
                [closest_pairs, min_dist] = PolygonToPolygonContactInfo(...
                    hand.finger_geometries{ind_finger}, obj.shape_vertices, finger_poses(:, ind_finger), obj.pose);
                %indices_pair_contact = (min_dist <= hand.finger_radius);
                %if (sum(indices_pair_contact) > 0)
               if (min_dist <= hand.finger_radius)
                    flag_contact(ind_finger) = 1;
                    %pt_contacts_finger = closest_pairs(3:4, indices_pair_contact);
                    pt_contacts_finger = closest_pairs(3:4, :);
                    vel_contacts_finger = bsxfun(@plus, finger_twists(1:2, ind_finger), ...
                        bsxfun(@times, finger_twists(3,ind_finger), [-pt_contacts_finger(2,:); pt_contacts_finger(1,:)]));            
                    %outward_normal_contacts_finger = closest_pairs(1:2, indices_pair_contact) - pt_contacts_finger;
                    outward_normal_contacts_finger = closest_pairs(1:2, :) - pt_contacts_finger;
                    outward_normal_contacts_finger = bsxfun(@rdivide, outward_normal_contacts_finger, sqrt(sum(outward_normal_contacts_finger.^2)) + eps);
                    pt_contacts = [pt_contacts, pt_contacts_finger];
                    vel_contacts = [vel_contacts, vel_contacts_finger];
                    outward_normal_contacts = [outward_normal_contacts, outward_normal_contacts_finger];
                end
           end
       end
      end
      
      function [twist_local, wrench_load_local, contact_mode] = ComputeVelGivenPointRoundFingerPush(obj, ...
              pt_global, vel_global, outward_normal_global, mu)
        % Input: 
        % contact point on the object (pt 2*1), pushing velocity (vel 2*1)
        % and outward normal (2*1, pointing from object to pusher) in world frame;  
        % mu: coefficient of friction. 
        % Note: Ensure that the point is actually in contact before using
        % this function. It does not check if point is on the object boundary.
        % Output: 
        % Body twist, friction wrench load (local frame) on the 1 level set of
        % limit surface and contact mode ('separation', 'sticking', 'leftsliding', 'rightsliding' ). 
        % Note that the third component is unnormalized,
        % i.e, F(3) is torque in Newton*Meter and V(3) is radian/second. 
        % If the velocity of pushing is breaking contact, then we return 
        % all zero 3*1 vectors. 
        
        % Change vel, pt and normal to local frame first. 
        vel_local = obj.GetVectorInLocalFrame(vel_global);        
        % Compute the point of contact.
        pt_local = obj.GetVectorInLocalFrame(pt_global - obj.pose(1:2));
        normal_local = obj.GetVectorInLocalFrame(outward_normal_global);
        
        [wrench_load_local, twist_local, contact_mode] = ComputeVelGivenSingleContactPtPush(vel_local, pt_local, ...
            normal_local, mu, obj.pho, obj.ls_coeffs, obj.ls_type);
        % Un-normalize the third components of F and V.
        wrench_load_local(3) = wrench_load_local(3) * obj.pho;
        twist_local(3) = twist_local(3) / obj.pho;  
        
      end
   end
end