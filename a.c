//Check a fixed portion of boundary points in the turn footprint for obstruction
if (CheckTurnFootprint(ThisWP_Yaw_UnitVec, currentWTP, false) == PU_NO_ERROR)
{
    //The turn has passed all the primary checks!
    bFeasibleTurn = true;
}
else
{
    //If this is a 180 degree turn then check if flipping the turn makes the turn footprint valid
 
    if (fabs(Simba::Utility::RestrictAngle(fTurnAngle_deg * M_PI / 180 - M_PI)) < 0.05)
    {
        currentWTP.nFlipYaxis *= -1;
        if (CheckTurnFootprint(ThisWP_Yaw_UnitVec, currentWTP, false) == PU_NO_ERROR)
        {
            bFeasibleTurn = true;
        }
    }
}
if (!bFeasibleTurn)
{
    CollectTurnAnalyzerData(ThisWP_Yaw_UnitVec, currentWTP);
    m_pLogger->LogStoreAndPrint("Path Planner::Rejecting Turn %s because violates footprint",tp->Name.c_str());
 
    continue;
}
else
{
    feasibleWTPs.push_back(currentWTP);
}
 
PlanUpdateError PathPlanner::CheckTurnFootprint(const Eigen::Vector2f& UnitVec, WaypointTurnPair wtp, bool bPrintToLog)
{
    //Check that the turn is entirely inside the waypoint's turn boxes
    //Check the boundary points for any obstructions
    bool bTurnIsOutsideKeepInBoxes = false;
    bool bFoundObstruction = false;
    PositionSystem::Coord UnitVec_ThisWP;
    int nBoundaryPointX_mm, nBoundaryPointY_mm;
    bool bValidBoundaryPt;
    IFeatureMap::eSurfType surfType;
    int nNullBoxes = 0;
    float dOffset = wtp.dOffset;
    Waypoint WP;
 
    WP_ERROR eWPError = m_WaypointsToPlan.GetWaypointAtIndex(wtp.nReferenceWaypointIndex, &WP);
    if (eWPError != WP_NO_ERROR)
    {
        if (bPrintToLog)
        {
            m_pLogger->LogStoreAndPrint("PathPlanner: Failed to find the matching waypoint. Index = %d, Count = %d",
                                        wtp.nReferenceWaypointIndex, m_WaypointsToPlan.Count());
        }
        return PU_LOGIC_ERROR;
    }
 
    int nFlipYaxis = wtp.nFlipYaxis;
    UnitVec_ThisWP.X = UnitVec(0);
    UnitVec_ThisWP.Y = UnitVec(1);
    Eigen::Matrix2d rotationMatrix;
    rotationMatrix << UnitVec_ThisWP.X, -UnitVec_ThisWP.Y,
                      UnitVec_ThisWP.Y, UnitVec_ThisWP.X;
 
    Eigen::Matrix<double, 2, 4> wpMatrix;
    wpMatrix << WP.dXpos_m, WP.dXpos_m, WP.dXpos_m, WP.dXpos_m,
                WP.dYpos_m, WP.dYpos_m, WP.dYpos_m, WP.dYpos_m;
 
    if (wtp.SegType == PlanSegmentType::SEG_TURN)
    {
        const ProfileType* tp = wtp.AssociatedTurn;
 
        for (size_t i = 0; i < tp->BoundingPolygon.size(); i++)
        {
            nBoundaryPointX_mm = static_cast<int>(std::round(1000 * (UnitVec_ThisWP.X * (tp->BoundingPolygon[i].X + dOffset) -
                                                UnitVec_ThisWP.Y * static_cast<float>(nFlipYaxis) * tp->BoundingPolygon[i].Y +
                                                WP.dXpos_m)));
            nBoundaryPointY_mm = static_cast<int>(std::round(1000 * (UnitVec_ThisWP.Y * (tp->BoundingPolygon[i].X + dOffset) +
                                                UnitVec_ThisWP.X * static_cast<float>(nFlipYaxis) * tp->BoundingPolygon[i].Y +
                                                WP.dYpos_m)));
 
            bValidBoundaryPt = false;
            for (int j = 0; j < WAYPOINT_TURNBOXES_COUNT; j++)
            {
                if (WP.TurnBox[j].xMin_mm == 0 && WP.TurnBox[j].xMax_mm == 0 &&
                    WP.TurnBox[j].yMin_mm == 0 && WP.TurnBox[j].yMax_mm == 0)
                {
                    nNullBoxes++;
                    continue;
                }
 
                if (WP.TurnBox[j].xMin_mm <= nBoundaryPointX_mm &&
                    WP.TurnBox[j].xMax_mm >= nBoundaryPointX_mm &&
                    WP.TurnBox[j].yMin_mm <= nBoundaryPointY_mm &&
                    WP.TurnBox[j].yMax_mm >= nBoundaryPointY_mm)
                {
                    bValidBoundaryPt = true;
                }
 
                if (bValidBoundaryPt)
                {
                    break;
                }
            }
 
            if (!bValidBoundaryPt && nNullBoxes < WAYPOINT_TURNBOXES_COUNT)
            {
                if (bPrintToLog)
                {
                    m_pLogger->LogStoreAndPrint("Rejecting Turn because footprint is outside keep-in boxes at (%d,%d)",
                                                nBoundaryPointX_mm, nBoundaryPointY_mm);
 
                    for (const auto& turnBox : WP.TurnBox)
                    {
                        if (!(turnBox.xMin_mm <= nBoundaryPointX_mm &&
                            turnBox.xMax_mm >= nBoundaryPointX_mm &&
                            turnBox.yMin_mm <= nBoundaryPointY_mm &&
                            turnBox.yMax_mm >= nBoundaryPointY_mm))
                        {
                            m_pLogger->LogStoreAndPrint("%s: Point [%i, %i] outside of turn box [%i, %i], [%i, %i]",
                                                        tp->Name.c_str(), nBoundaryPointX_mm, nBoundaryPointY_mm,
                                                        turnBox.xMin_mm, turnBox.xMax_mm,
                                                        turnBox.yMin_mm, turnBox.yMax_mm);
                        }
                    }
 
                }