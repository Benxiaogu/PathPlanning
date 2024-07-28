classdef PriorityQueue < handle
    % PriorityQueue defined
    properties
        elements = {};
        priorities = [];
    end
    
    methods
        function insert(obj, element, priority)
            obj.elements{end + 1} = element;
            obj.priorities(end + 1) = priority;
            obj.bubbleUp(length(obj.priorities));
        end
        
        function [element, priority] = extractMin(obj)
            if isempty(obj.elements)
                error('PriorityQueue is empty');
            end
            element = obj.elements{1};
            priority = obj.priorities(1);
            obj.elements{1} = obj.elements{end};
            obj.priorities(1) = obj.priorities(end);
            obj.elements(end) = [];
            obj.priorities(end) = [];
            obj.bubbleDown(1);
        end
        
        function isEmpty = isEmpty(obj)
            isEmpty = isempty(obj.elements);
        end
        
        function bubbleUp(obj, index)
            while index > 1
                parentIndex = floor(index / 2);
                if obj.priorities(index) < obj.priorities(parentIndex)
                    obj.swap(index, parentIndex);
                    index = parentIndex;
                else
                    break;
                end
            end
        end
        
        function bubbleDown(obj, index)
            while true
                leftChildIndex = 2 * index;
                rightChildIndex = 2 * index + 1;
                smallestIndex = index;
                
                if leftChildIndex <= length(obj.priorities) && obj.priorities(leftChildIndex) < obj.priorities(smallestIndex)
                    smallestIndex = leftChildIndex;
                end
                
                if rightChildIndex <= length(obj.priorities) && obj.priorities(rightChildIndex) < obj.priorities(smallestIndex)
                    smallestIndex = rightChildIndex;
                end
                
                if smallestIndex ~= index
                    obj.swap(index, smallestIndex);
                    index = smallestIndex;
                else
                    break;
                end
            end
        end
        
        function swap(obj, index1, index2)
            tempElement = obj.elements{index1};
            tempPriority = obj.priorities(index1);
            obj.elements{index1} = obj.elements{index2};
            obj.priorities(index1) = obj.priorities(index2);
            obj.elements{index2} = tempElement;
            obj.priorities(index2) = tempPriority;
        end
    end
end
