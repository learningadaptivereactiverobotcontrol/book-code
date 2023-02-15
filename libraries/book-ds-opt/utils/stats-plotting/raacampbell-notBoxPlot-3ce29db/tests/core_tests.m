classdef core_tests < matlab.unittest.TestCase
    % Unit tests for notBoxPlot

    properties

    end %properties


    methods (Test)


        function checkInputParameters(testCase)
            %Check that no gross errors are produced for all param/val pairs not already tested above
            y=rand(10,3);
            x=[1,2,2]; %To place to boxes on the same x location and one on its own

            clf
            H=notBoxPlot(y,x,'jitter',0.6,'style','sdline');

            clf
            notBoxPlot(y,[],'style','patch','interval','SEM');     

            clf
            notBoxPlot(y,x,'interval','tInterval');  

            clf
            h=notBoxPlot(y,x,'interval','tInterval','markMedian',false);  
            testCase.verifyFalse(isfield(h,'med'))

            clf
            h=notBoxPlot(y,x,'interval','tInterval','markMedian',true);
            testCase.verifyTrue(isfield(h,'med'))

            clf
            h=notBoxPlot(y,x,'style','line','markMedian',false);  
            testCase.verifyFalse(isfield(h,'med'))

            clf
            h=notBoxPlot(y,x,'style','line','markMedian',true);
            testCase.verifyTrue(isfield(h,'med'))

            %With NaNs
            clf
            y(1,1)=nan;
            H=notBoxPlot(y,x,'jitter',0.6,'style','sdline');

            %Check that the showCase example runs
            NBP.showCase
            
            close(gcf)
        end


        function checkOutputs(testCase)
            %Check output args
            y=rand(10,3);
            x=[1,2,2]; %To place to boxes on the same x location and one on its own

            clf
            [H,stats]=notBoxPlot(y,x,'jitter',0.6,'style','sdline');
            testCase.verifyTrue(isstruct(H))
            testCase.verifyTrue(isstruct(stats))
            testCase.verifyTrue(length(H)==length(stats))

            clf
            [H,stats]=notBoxPlot(y,x,'jitter',0.6,'markMedian',true);

            testCase.verifyTrue(isfield(stats,'median'))

            
            close(gcf)
        end
    end %methods (Test)

end %classdef core_tests < matlab.unittest.TestCase