# Title     : TODO
# Objective : TODO
# Created by: a1091793
# Created on: 21/12/17

library(ggplot2)

printParetoFront <- function(pop_file, pdf_file, xlab, ylab)
{
    pop <- readPopulation(pop_file)
    pop[[2]]$Obj2 <- pop[[2]]$Obj2 / 1E6
    pop[[2]]$solnid <- as.character(rownames(pop[[2]]))
    pdf(pdf_file, 6, 4, useDingbats = FALSE)
    plot <- ggplot(data=pop[[2]], mapping=aes(x=Obj1, y=Obj2, label=solnid)) + geom_point(shape=21) + labs(x=xlab, y=ylab) + theme_minimal()
    print(plot)
    dev.off()
}

